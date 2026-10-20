"""Ordered real-camera history; capture deadlines belong to the physics publisher."""
from __future__ import annotations
import asyncio
import base64
from dataclasses import asdict
import gzip
import hashlib
import json
import time
from collections import deque

from aiohttp import ClientSession, ClientTimeout, WSMsgType

CAPACITY = 1024


async def ordered_io(function,*args):
    """Finish an in-flight file operation before cancellation can close its stream."""
    task=asyncio.create_task(asyncio.to_thread(function,*args))
    try:return await asyncio.shield(task)
    except asyncio.CancelledError:
        await task
        raise


def classify(data):
    text=json.dumps(data,ensure_ascii=False).lower()
    if data.get('rc')!=0 and ('apos' in text and ('stop' in text or 'conflict' in text)):
        return '模型语义冲突／本地安全拒绝'
    return '模型请求失败' if data.get('rc')!=0 else (data.get('decode_info') or {}).get('decode_reason','')


class OrderedHistory:
    def __init__(self, simulation, output, instruction, check):
        self.sim,self.output,self.instruction,self.check=simulation,output,instruction,check
        self.frames=deque();self.sequence=0;self.sample_count=0
        self.failure='';self.waiter=None;self.socket=None;self.last_frame=None;self.predict_after=0
        self.client='dm-h65-auto-'+output.name+'-'+str(time.time_ns())
        self.requests=gzip.open(output/'requests.jsonl.gz','xt')
        self.responses=(output/'responses.jsonl').open('x',buffering=1)
        self.captures=(output/'captures.jsonl').open('x',buffering=1)
        self.tasks=[];self.closed=False;self.publisher_token=None
        self.epoch=getattr(simulation, '_scene_epoch', 0);self.phase='SESSION_CONNECT'
        self.write_queue=asyncio.Queue(maxsize=CAPACITY);self.writer_task=None

    def fail(self,error):
        if not self.failure:self.failure=str(error)
        if self.waiter and not self.waiter.done():self.waiter.set_exception(RuntimeError(self.failure))

    def offer(self,frame,*,published=False):
        if frame is None:raise RuntimeError('MISSING_OR_WRONG_EPOCH_FRAME')
        # A publisher-captured frame may wait in the FIFO during a slow consumer.
        # Its real capture time is retained; it is not a newly sampled old image.
        if not published and time.monotonic_ns()-frame.monotonic_ns>150_000_000:
            raise RuntimeError('STALE_CAMERA_FRAME')
        if self.last_frame and (frame.stamp_ns<=self.last_frame.stamp_ns or frame.monotonic_ns<=self.last_frame.monotonic_ns):
            raise RuntimeError('DUPLICATE_OR_UNORDERED_FRAME')
        if len(self.frames)>=CAPACITY or self.write_queue.full():raise RuntimeError('HISTORY_FIFO_OVERFLOW')
        self.sample_count+=1
        record={k:v for k,v in asdict(frame).items() if k not in {'jpeg','rgb'}}
        record.update(sample=self.sample_count,file=f'frame-{self.sample_count:06d}.jpg',
                      jpeg_sha256=hashlib.sha256(frame.jpeg).hexdigest())
        self.write_queue.put_nowait((record,frame.jpeg))
        self.frames.append((self.sample_count,frame));self.last_frame=frame

    def _write_capture(self,record,jpeg):
        (self.output/record['file']).write_bytes(jpeg)
        self.captures.write(json.dumps(record)+'\n')

    async def writer(self):
        while True:
            item=await self.write_queue.get()
            try:
                if item is None:return
                await asyncio.to_thread(self._write_capture,*item)
            except Exception as error:
                self.fail('HISTORY_RECORDING_FAILED:'+str(error))
            finally:self.write_queue.task_done()

    async def sampler(self):
        try:
            while not self.closed:
                self.check()
                batch=await asyncio.to_thread(self.sim.drain_history_capture,self.publisher_token)
                for frame in batch:self.offer(frame,published=True)
                await asyncio.sleep(.01)
        except asyncio.CancelledError:raise
        except Exception as error:self.fail(error)

    def _write_request(self,request):
        self.requests.write(json.dumps(request)+'\n');self.requests.flush()

    def _write_response(self,record):
        self.responses.write(json.dumps(record)+'\n')

    async def exchange(self,action,data,timeout):
        self.check();data['request_id']=f'{self.client}:{self.epoch}:{action}:{data.get("seq",0)}'
        request={'action':action,'data':data}
        await ordered_io(self._write_request,request)
        self.check()
        start=time.monotonic();await self.socket.send_json(request)
        while True:
            self.check()
            if time.monotonic()-start>timeout:raise RuntimeError('MODEL_REQUEST_TIMEOUT:'+action)
            try:message=await asyncio.wait_for(self.socket.receive(),timeout=.1)
            except asyncio.TimeoutError:continue
            if message.type!=WSMsgType.TEXT:raise RuntimeError('MODEL_DISCONNECTED')
            elapsed=time.monotonic()-start
            await ordered_io(self._write_response,{'received_monotonic_ns':time.monotonic_ns(),
                'elapsed_s':elapsed,'raw_response':message.data})
            reply=json.loads(message.data);returned=reply.get('data',{})
            if reply.get('action')!=action or returned.get('request_id')!=data['request_id']:
                raise RuntimeError('MODEL_REQUEST_CORRELATION_MISMATCH')
            return reply,elapsed

    async def consumer(self):
        try:
            async with ClientSession(timeout=ClientTimeout(total=None,sock_connect=3.)) as session:
                async with session.ws_connect('ws://127.0.0.1:8050',heartbeat=10.,max_msg_size=64*1024**2) as ws:
                    self.socket=ws
                    for action,data in [('login',{'clientId':self.client}),('reset',{})]:
                        reply,_=await self.exchange(action,data,3.)
                        if reply['data'].get('rc')!=0:raise RuntimeError('MODEL_SESSION_REJECTED')
                    self.phase='BUFFERING'
                    while not self.closed:
                        self.check()
                        if not self.frames:
                            await asyncio.sleep(.005);continue
                        sample,frame=self.frames.popleft();self.sequence+=1
                        predict=bool(self.waiter and not self.waiter.done() and not self.frames
                                     and self.sequence>=4 and frame.monotonic_ns>=self.predict_after
                                     and time.monotonic_ns()-frame.monotonic_ns<=150_000_000)
                        if predict:self.sim.begin_motion_guard()
                        self.phase='INFERENCE' if predict else 'BUFFERING'
                        data={'seq':self.sequence,'image':base64.b64encode(frame.jpeg).decode(),
                              'instruction':self.instruction if predict else '', 'diagnostics':predict}
                        reply,elapsed=await self.exchange('next',data,10. if predict else 3.)
                        returned=reply['data']
                        if returned.get('seq')!=self.sequence:raise RuntimeError('MODEL_SEQUENCE_MISMATCH')
                        if not predict:
                            if returned.get('rc')!=0 or 'actions' in returned or returned.get('msg')!='image received':
                                raise RuntimeError('MODEL_BUFFER_ACK_INVALID')
                        elif not self.waiter.done():
                            self.waiter.set_result({'reply':reply,'frame':frame,'elapsed_s':elapsed,
                                                    'sample':sample,'sequence':self.sequence})
                        self.phase='BUFFERING'
        except asyncio.CancelledError:raise
        except Exception as error:self.fail(error)

    async def start(self):
        self.publisher_token=self.sim.begin_history_capture(capacity=CAPACITY)
        self.writer_task=asyncio.create_task(self.writer())
        self.tasks=[asyncio.create_task(self.sampler()),asyncio.create_task(self.consumer())]

    async def predict(self):
        if self.failure:raise RuntimeError(self.failure)
        if self.waiter and not self.waiter.done():raise RuntimeError('PREDICTION_ALREADY_PENDING')
        self.waiter=asyncio.get_running_loop().create_future();self.predict_after=time.monotonic_ns()
        start=time.monotonic()
        try:
            while not self.waiter.done():
                self.check()
                if self.failure:raise RuntimeError(self.failure)
                if time.monotonic()-start>13.:raise RuntimeError('HISTORY_DRAIN_OR_PREDICTION_TIMEOUT')
                await asyncio.sleep(.01)
            return self.waiter.result()
        finally:
            if self.waiter and not self.waiter.done():self.waiter.cancel()

    async def close(self):
        if self.closed:return
        self.closed=True
        if self.publisher_token is not None:
            self.sim.end_history_capture(self.publisher_token);self.publisher_token=None
        for task in self.tasks:task.cancel()
        for task in self.tasks:
            try:await task
            except asyncio.CancelledError:pass
        if self.waiter and not self.waiter.done():self.waiter.cancel()
        if self.writer_task:
            await self.write_queue.join();await self.write_queue.put(None);await self.writer_task
        else:
            while not self.write_queue.empty():
                item=self.write_queue.get_nowait()
                await asyncio.to_thread(self._write_capture,*item);self.write_queue.task_done()
        await asyncio.to_thread(self.requests.close)
        await asyncio.to_thread(self.responses.close)
        await asyncio.to_thread(self.captures.close)
