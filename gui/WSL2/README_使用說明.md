# Hunter Autoware-Lite Online v1.5 — Waypoint 導航與 RViz

**系統**：Windows 11 + WSL2 Ubuntu 18.04 + ROS Melodic + Hunter + VLP-16 + Autoware.AI。

## 這一版新增什麼

- **00 設備總覽**、**01 TF**、**02 NDT Mapping**、**03 NDT Matching**、**05 ROS Bag**：保留已驗證的流程。
- **04 路徑測試**：直接讀取 `x,y,z,yaw,velocity,change_flag` CSV，使用 `autoware_msgs/Lane` 發布 `/offline_nav/final_waypoints`，每秒更新 10 次。不再依賴 `waypoint_loader` 的 `_multi_lane_csv` 設定，解決原本 `Waypoint=0` 的常見來源。路徑速度 CSV 單位為 **km/h**，發布為 **m/s**。
- **導航 RViz**：PCD 地圖 `/points_map`、即時 VLP-16 `/velodyne_points`、NDT `/ndt_pose`、Hunter Marker、NDT 走過的路徑（黃色）、目標 Waypoint（綠色）和終點（紅色）。Fixed Frame 為 `map`。
- **監督式路徑測試**：`hunter_nav_supervisor.py` 預設 **Dry Run、不發布 `/cmd_vel`**；GUI 特別授權後才會以 `--hardware` 啟用單一受限制的 `/cmd_vel` Publisher。操作員要一直按住 GUI 的路徑跟隨按鈕，放開後本輪測試即停止，若要重來需重新啟動 supervisor。

## 重要安全邊界

**不是已通過安全認證的自主導航系統；不能無人看管、不能用於有人的通道或真實道路。** 即使具備前方 LiDAR ROI 候選障礙點檢測，VLP-16 的視線盲區、安裝高度、地面誤判、玻璃及細小障礙物仍可能無法可靠偵測。**此功能不是完整防撞／障礙物避讓**。

首次實車使用前必須在隔離、清空、無人、平坦的場地，由專人監看並實際驗證：

1. **獨立實體急停**正常。
2. **底盤失去連續命令／WSL2 或 USB-CAN 突然斷線時能可靠停車**。車體驅動程式的軟體 STOP 不能代替這個驗證。
3. 遙控器可以正確切換 **CAN 命令控制（`control_mode=1`）**及手動接管；沒有把車停穩前不可切換。
4. 室內地面、光達與 `base_link` 外參已實測，`map → base_link → velodyne` 的 TF 正確。
5. `/ndt_pose` 穩定、`/ndt_stat` 的 score/iteration/exe_time 實測合理、RViz PCD 與 Live LiDAR 對齊。
6. Waypoint CSV 通過 GUI「檢查 CSV / 高度跳動」；**不得跳過安全檢查、修改門檻以讓明顯不合理路徑通過**。
7. 起點與車輛當前估測位置相近，起始車頭朝向吻合；不可在路線終點直接啟動同一條正向路徑。

**你目前貼出的 CSV 前四筆 Z 值約從 -0.60 到 -0.05m，而 XY 約移動 1.6m。** 如果測試場地平坦，這表示高度可能漂移。新版預檢會拒絕高變化/高坡度資料。應先修正 TF 或定位，重新低速遙控記錄良好路徑，不要任意把 Z 改成零掩蓋問題。

## 安裝

1. 先停止 Hunter 實車控制，關閉 v1.4 WSL2 GUI。先正常停止 Bag；Windows Device Bridge 不用重裝，可保持 Relay 運作。
2. 下載 `Hunter_WSL2_Navigation_v1.5_Update.zip` 到 Windows `Downloads`。
3. WSL2 中執行：

```bash
mkdir -p /tmp/hunter_v15
unzip -o /mnt/c/Users/Cobe/Downloads/Hunter_WSL2_Navigation_v1.5_Update.zip 'WSL2/*' -d /tmp/hunter_v15
cp ~/hunter_online_gui/hunter_online_gui.py ~/hunter_online_gui/hunter_online_gui_v1.4_backup.py
cp -a /tmp/hunter_v15/WSL2/. ~/hunter_online_gui/
python3 -m py_compile ~/hunter_online_gui/*.py
python3 ~/hunter_online_gui/hunter_online_gui.py
```

`~/hunter_online_gui/bags/`、`trajectories/`、原本 PCD 檔案不會被覆蓋。

## 操作順序

1. **Windows Bridge**：USB-CAN Attached、WSL2 IP 正確、UDP Relay Forwarded 增加。
2. **00**：ROS Master、can0、Hunter Driver、VLP-16 Driver 正常。
3. **01**：設定正確的 LiDAR TF；儲存 PCD 檔案路徑。
4. **03**：啟動 Live NDT Matching，載入先前建好的 PCD；設定初始位姿；確認地圖與點雲在 RViz 對齊。
5. **04**：按 **「檢查 CSV / 高度跳動」**。如果提示 Z 高度跳動、間距過大、行進方向不合、終點速度非零，**停止，先重新記錄正確路線**。
6. 按 **「載入 Waypoint」**，確認 `/hunter_nav/route` 和 `/offline_nav/final_waypoints`。
7. 按 **「啟動 Pure Pursuit」**，再按 **「檢查路徑與 Twist」**；應有 `/offline_nav/twist_cmd`，不再是 `Waypoint=0`。
8. 按 **「開啟導航 RViz」**：綠線是規劃路徑；黃線是 NDT 估測軌跡；紅箭頭是目的地。
9. **先按「① Dry Run」**，按住授權按鈕時觀察 `/hunter_nav/safety_state` 和 `/hunter_nav/applied_cmd`。Dry Run 完全不發布 `/cmd_vel`。放開後重新啟動 supervisor 才能再試。
10. **只有上述全部合格、完成實體安全驗證後**才可勾選確認並按「② 準備實車路徑測試」。GUI 會要求二次授權；須維持 `control_mode=1`。一直按住操作按鈕時才允許 supervisor 發送**最高 0.10 m/s 線速度、±0.08 rad/s 角速度**到 `/cmd_vel`；放開後送零速度並結束本輪測試，重新開始需從路線起點再檢查。
11. 任何異常立即使用**實體急停**，不要只依賴 GUI STOP。

## ROS Topics

| Topic | Message | 用途 |
|---|---|---|
| `/ndt_pose` | `geometry_msgs/PoseStamped` | NDT 定位 |
| `/ndt_stat` | `autoware_msgs/NDTStat` | fitness score、iteration、時間 |
| `/hunter_status` | `hunter_msgs/HunterStatus` | 底盤故障碼、命令模式、速度 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 即時點雲；輔助前方障礙候選檢測 |
| `/offline_nav/final_waypoints` | `autoware_msgs/Lane` | Pure Pursuit 消費的路徑 |
| `/offline_nav/twist_cmd` | `geometry_msgs/TwistStamped` | 控制器計算的路徑命令 |
| `/hunter_nav/route` | `nav_msgs/Path` | RViz 綠色路徑及 Supervisor 路徑驗證 |
| `/hunter_nav/goal` | `geometry_msgs/PoseStamped` | RViz 終點 |
| `/hunter_nav/operator_hold` | `std_msgs/Bool` | GUI 活性授權，沒有新訊息即過期 |
| `/hunter_nav/safety_state` | `std_msgs/String` | STOP、DISARMED、ACTIVE、GOAL 理由 |
| `/hunter_nav/applied_cmd` | `geometry_msgs/TwistStamped` | 控制命令監控；Dry Run 無馬達輸出 |
| `/cmd_vel` | `geometry_msgs/Twist` | **僅授權 Hardware 模式輸出** |

## 預設軟體限制

- 始終只允許前進；不得倒退或零速原地旋轉。
- NDT 位姿、NDTStat、Pure Pursuit Twist、Hunter Status、LiDAR 與操作員授權任何一個逾時即停車。
- NDT fitness >預設 2.0（需現地驗證）、iterations >=29、計算時間 >150ms 時禁止實車指令。
- 車身距離路徑 XY >0.45m、方向誤差 >0.70rad、NDT 突跳 >0.60m，停止。
- 起點距離必須 <=0.50m 且 yaw 誤差 <=0.40rad。
- 到終點 0.55m 內停止。
- 若任何其他 Publisher 發送 `/cmd_vel`，停止並鎖定本輪測試。
- 總控最大速度 0.10m/s、角速度 ±0.08rad/s，並限制加速度。
- GUI 失焦、放開按住按鈕、Python GUI 不再送出 heartbeat 時，授權自動過期；supervisor 處理停車。但 WSL2 整體當機時**僅能依賴車體自身的硬體通訊逾時／急停**。

## 故障排除

- `Pure Pursuit waiting: Waypoint=0`：確認現在是新版 `offline_nav_bridge.py --csv ...`，日誌應有 `Validated route direct CSV`。檢查 `/offline_nav/final_waypoints` 與 `/hunter_nav/route` 存在；先停止舊 nav 橋接程序。
- `STOP: NDT fitness score ...`：定位品質不合格，先檢查 PCD、初始位姿、外參、時間同步；不要只放寬 threshold。
- `STOP: Hunter 尚非 AUTONOMOUS`：確認底盤回報 `control_mode=1`（版本有差異時依本車使用手冊及狀態定義確認）。
- `STOP: LiDAR`：檢查 Windows Relay、TF、即時點雲是否存在，補充安全感測配置。
- `STOP: 操作員放開`：預期行為。按「停止導航監督節點」後重啟 Dry Run／Hardware，再重新檢查起點。
- RViz 沒有綠線：`rostopic echo -n 1 /hunter_nav/route`；沒紅色終點：`rostopic echo -n 1 /hunter_nav/goal`。

**備註**：此更新尚未在使用者的實際 Hunter + WSL2 環境中執行，所有車體行為與安全停車條件仍需現場驗證。
