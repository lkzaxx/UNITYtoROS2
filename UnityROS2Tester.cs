using System;
using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;

/// <summary>
/// Unity-ROS2 連線測試腳本
/// 
/// 使用方法：
/// 1. 將此腳本附加到 Unity 場景中的 GameObject
/// 2. 確保已安裝 ROS-TCP-Connector 套件
/// 3. 設定 ROS Settings: IP=127.0.0.1, Port=10000
/// 4. 運行場景並觀察 Console 輸出
/// </summary>
public class UnityROS2Tester : MonoBehaviour
{
    [Header("連線設定")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("測試設定")]
    public float testInterval = 2.0f;
    public bool enableHeartbeatTest = true;
    public bool enablePoseTest = true;
    public bool enableJointTest = true;

    [Header("測試資料")]
    public Vector3 testPosition = new Vector3(0.5f, 0.0f, 0.3f);
    public Vector3 testRotation = Vector3.zero;
    public float[] testJointPositions = new float[7] { 0f, 0.5f, 0f, -1f, 0f, 0.5f, 0f };

    // ROS 連接器
    private ROSConnection ros;

    // 測試狀態
    private bool isConnected = false;
    private int heartbeatCount = 0;
    private int jointStateCount = 0;
    private float lastTestTime = 0f;

    // 主題名稱
    private const string HEARTBEAT_TOPIC = "/unity/heartbeat";
    private const string POSE_TOPIC = "/unity/pose";
    private const string JOINT_CMD_TOPIC = "/unity/joint_commands";
    private const string JOINT_STATE_TOPIC = "/openarm/joint_states";
    private const string PING_SERVICE = "/unity/ping";

    void Start()
    {
        Debug.Log("🚀 開始 Unity-ROS2 連線測試");

        // 初始化 ROS 連接
        InitializeROS();

        // 開始測試協程
        StartCoroutine(RunTests());
    }

    void InitializeROS()
    {
        try
        {
            // 獲取 ROS 連接器實例
            ros = ROSConnection.GetOrCreateInstance();

            // 設定連接參數
            ros.ConnectOnStart = true;

            Debug.Log($"📡 嘗試連接到 ROS 2: {rosIPAddress}:{rosPort}");

            // 註冊訂閱者
            if (enableHeartbeatTest)
            {
                ros.Subscribe<StringMsg>(HEARTBEAT_TOPIC, OnHeartbeatReceived);
                Debug.Log($"📥 已訂閱: {HEARTBEAT_TOPIC}");
            }

            ros.Subscribe<JointStateMsg>(JOINT_STATE_TOPIC, OnJointStateReceived);
            Debug.Log($"📥 已訂閱: {JOINT_STATE_TOPIC}");

            // 註冊發布者
            if (enablePoseTest)
            {
                ros.RegisterPublisher<PoseStampedMsg>(POSE_TOPIC);
                Debug.Log($"📤 已註冊發布者: {POSE_TOPIC}");
            }

            if (enableJointTest)
            {
                ros.RegisterPublisher<JointStateMsg>(JOINT_CMD_TOPIC);
                Debug.Log($"📤 已註冊發布者: {JOINT_CMD_TOPIC}");
            }

            isConnected = true;
            Debug.Log("✅ ROS 連接初始化完成");

        }
        catch (Exception e)
        {
            Debug.LogError($"❌ ROS 連接初始化失敗: {e.Message}");
            isConnected = false;
        }
    }

    IEnumerator RunTests()
    {
        // 等待連接建立
        yield return new WaitForSeconds(2f);

        while (true)
        {
            if (isConnected && Time.time - lastTestTime >= testInterval)
            {
                // 執行測試
                RunAllTests();
                lastTestTime = Time.time;
            }

            yield return new WaitForSeconds(0.1f);
        }
    }

    void RunAllTests()
    {
        Debug.Log("🧪 執行連線測試...");

        // 測試 Ping 服務
        TestPingService();

        // 測試姿態發布
        if (enablePoseTest)
        {
            TestPosePublishing();
        }

        // 測試關節命令發布
        if (enableJointTest)
        {
            TestJointCommandPublishing();
        }

        // 顯示統計資訊
        ShowStatistics();
    }

    void TestPingService()
    {
        try
        {
            // 注意：這裡需要根據實際的服務類型調整
            // ros.SendServiceMessage<TriggerRequest, TriggerResponse>(PING_SERVICE, new TriggerRequest(), OnPingResponse);
            Debug.Log("📞 Ping 服務測試 (需要實作服務呼叫)");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"⚠️ Ping 服務測試失敗: {e.Message}");
        }
    }

    void TestPosePublishing()
    {
        try
        {
            var poseMsg = new PoseStampedMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeMsg
                    {
                        sec = (uint)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                        nanosec = 0
                    },
                    frame_id = "unity_world"
                },
                pose = new PoseMsg
                {
                    position = testPosition.To<FLU>(),
                    orientation = Quaternion.Euler(testRotation).To<FLU>()
                }
            };

            ros.Publish(POSE_TOPIC, poseMsg);
            Debug.Log($"📍 發布姿態: 位置={testPosition}, 旋轉={testRotation}");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ 姿態發布失敗: {e.Message}");
        }
    }

    void TestJointCommandPublishing()
    {
        try
        {
            var jointMsg = new JointStateMsg
            {
                header = new HeaderMsg
                {
                    stamp = new TimeMsg
                    {
                        sec = (uint)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                        nanosec = 0
                    },
                    frame_id = "openarm_base_link"
                },
                name = new string[] { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7" },
                position = testJointPositions,
                velocity = new double[7],
                effort = new double[7]
            };

            ros.Publish(JOINT_CMD_TOPIC, jointMsg);
            Debug.Log($"🦾 發布關節命令: [{string.Join(", ", testJointPositions)}]");
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ 關節命令發布失敗: {e.Message}");
        }
    }

    void OnHeartbeatReceived(StringMsg heartbeat)
    {
        heartbeatCount++;
        Debug.Log($"💓 收到心跳 #{heartbeatCount}: {heartbeat.data}");
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        jointStateCount++;

        if (jointState.position != null && jointState.position.Length > 0)
        {
            string positions = string.Join(", ", Array.ConvertAll(jointState.position, x => x.ToString("F3")));
            Debug.Log($"🦾 收到關節狀態 #{jointStateCount}: [{positions}]");
        }
    }

    void ShowStatistics()
    {
        Debug.Log($"📊 統計資訊 - 心跳: {heartbeatCount}, 關節狀態: {jointStateCount}");
    }

    void OnGUI()
    {
        // 在螢幕上顯示連線狀態
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("Unity-ROS2 連線測試", GUI.skin.box);

        GUILayout.Label($"連線狀態: {(isConnected ? "✅ 已連接" : "❌ 未連接")}");
        GUILayout.Label($"ROS 地址: {rosIPAddress}:{rosPort}");
        GUILayout.Label($"心跳計數: {heartbeatCount}");
        GUILayout.Label($"關節狀態計數: {jointStateCount}");

        if (GUILayout.Button("手動測試"))
        {
            RunAllTests();
        }

        if (GUILayout.Button("重新連接"))
        {
            InitializeROS();
        }

        GUILayout.EndArea();
    }

    void OnDestroy()
    {
        // 清理資源
        if (ros != null)
        {
            Debug.Log("🔌 斷開 ROS 連接");
        }
    }
}

/*
使用說明：

1. Unity 設定：
   - 安裝 ROS-TCP-Connector 套件
   - Window > ROS Settings 設定 ROS IP Address: 127.0.0.1, ROS Port: 10000
   - 將此腳本附加到場景中的 GameObject

2. ROS 2 端設定：
   - 確保 Docker 容器正在運行
   - 啟動 TCP Endpoint: ros2 run ros_tcp_endpoint default_server_endpoint
   - 啟動橋接節點: ros2 run unity_openarm_bridge tcp_bridge_node

3. 測試流程：
   - 運行 Unity 場景
   - 觀察 Console 輸出
   - 檢查 ROS 2 端是否收到訊息: ros2 topic echo /unity/pose

4. 預期結果：
   - Unity Console 顯示心跳和關節狀態訊息
   - ROS 2 端收到 Unity 發送的姿態和關節命令
   - GUI 顯示連線狀態和統計資訊
*/
