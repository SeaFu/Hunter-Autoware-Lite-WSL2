# Hunter Online GUI v1.5.8

完整操作請見 [雙 GUI 教學](../docs/GUI_雙平台操作教學.md)。

- `Windows_Bridge/hunter_bridge_gui.ps1`：Windows WinForms GUI，USB-CAN 掛載、WSL IP、VLP-16 UDP Relay、安裝／開啟 WSL GUI。
- `WSL2/hunter_online_gui.py`：WSL2 PyQt5 GUI，包含 CAN/ROS Driver、NDT 建圖／定位、Waypoint／Pure Pursuit、RViz、Bag。
- `WSL2/hunter_motion_test.py`：GUI 00 的短距離人工動作測試工具，已與 GUI 一起打包，不再依賴 `~/hunter_motion_test.py`。
- `WSL2/launch/`、`WSL2/*.rviz`：隨 GUI 一併安裝的 ROS launch 和 RViz 設定。

**安裝方法**：Windows Bridge 的「安裝／更新 WSL GUI」會將本資料夾同層 `WSL2/` 複製到 Linux `~/hunter_online_gui/`，因此 Windows Bridge 和 WSL2 兩個資料夾不能分開放置。

**本程式碼不含** WSL 自訂核心、完整 ROS Melodic、Hunter SDK、Autoware.AI。既有的方式 A/B CAN 核心說明仍照原 GitHub README 保留。
