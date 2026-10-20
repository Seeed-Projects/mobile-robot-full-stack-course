const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
function fixture(){
 const elements=new Map();const el=id=>{if(!elements.has(id))elements.set(id,{textContent:'',value:'',style:{},classList:{toggle(){},contains(){return false;}},setAttribute(){}});return elements.get(id);};
 const sandbox={document:{getElementById:el,querySelector:el,querySelectorAll:()=>[],documentElement:{},addEventListener(){}},localStorage:{getItem:()=>null,setItem(){}},fetch:async()=>({json:async()=>({polygons:[]})}),setInterval(){},window:{addEventListener(){}},Date:{now:()=>100000},console};
 const source=fs.readFileSync(path.join(__dirname,'../vln_mujoco/sim_demo_web/app.js'),'utf8').replace("connect();h264('first');h264('third');requestAnimationFrame(liveDraw);",'');
 vm.createContext(sandbox);vm.runInContext(source+`;globalThis.setState=x=>{state=x;rememberModelPoints();};globalThis.setLang=x=>{language=x;};globalThis.status=targetStatus;globalThis.points=()=>overlayPoints;globalThis.draws=[];drawPointAt=(ctx,x,y,color,label)=>draws.push({x,y,label});globalThis.draw=meta=>{draws.length=0;drawModelPointOverlay({},480,270,meta);return draws;};`,sandbox);
 return sandbox;
}
const camera={camera_position:[0,0,0],camera_orientation:[1,0,0,0,1,0,0,0,1],camera_fovy:90};
function prediction(index=1){return {navigation:{task_id:'one'},prediction:{task_id:'one',index,capture_stamp_ns:'99000000000',data:{pointing:{opos_px:[20,20]}},display_anchors:{opos_px:{world_point:[0,0,-2]}}}};}
test('live point follows current camera instead of copying input pixels',()=>{
 const f=fixture();f.setState(prediction());let points=f.draw(camera);assert.equal(points.length,1);assert.equal(points[0].x,240);
 points=f.draw({...camera,camera_position:[1,0,0]});assert.ok(points[0].x<200);
});
test('missing OPOS is explicitly labelled; history expires and reset clears',()=>{
 const f=fixture();f.setState(prediction());const next=prediction(2);next.prediction.data.pointing.opos_px=null;next.prediction.display_anchors={};f.setState(next);
 assert.match(f.status(),/当前预测未见/);assert.match(f.draw(camera)[0].label,/上次可见/);
 f.setLang('en');assert.match(f.status(),/not visible/);assert.match(f.draw(camera)[0].label,/last seen/);
 f.Date.now=()=>111000;assert.equal(f.draw(camera).length,0);
 f.setState({navigation:null,prediction:null});assert.equal(f.points().opos,null);assert.equal(f.draw(camera).length,0);
});
test('points outside the camera and raw pixels without depth are not false targets',()=>{
 const f=fixture();let p=prediction();p.prediction.display_anchors.opos_px.world_point=[0,0,2];f.setState(p);assert.equal(f.draw(camera).length,0);
 p=prediction();p.prediction.task_id='two';p.navigation.task_id='two';p.prediction.display_anchors={};f.setState(p);assert.equal(f.draw(camera).length,0);
});
