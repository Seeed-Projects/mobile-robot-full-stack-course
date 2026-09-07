# DM-H65 底盘 WebUI

正式后端位于 `src/dm_h65/webui.py`，本目录的 `server.py` 是源码开发兼容入口；React 源码位于 `frontend/`。

安装 SDK 后直接运行：

```bash
dm-h65-webui --host 0.0.0.0 --port 8765 --interface can0
```

源码开发模式：

```bash
python webui/server.py --simulate
```

前端构建：

```bash
cd webui/frontend
npm install
npm run build
cd ../..
python scripts/sync_webui_assets.py
```

完整的启动、远程连接、操作和安全说明见 [`../docs/WEBUI.md`](../docs/WEBUI.md)。
