using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using Google.Protobuf.Collections;
using System.Linq;
using System.Collections.Generic;


[RequireComponent(typeof(JointDriveController))]
public class SnakeAgent : Agent
{
    const float m_MaxWalkingSpeed = 10; //The max walking speed
    [Header("Body Parts")] public Transform bodySegment0;
    public Transform bodySegment1;
    public Transform bodySegment2;
    public Transform bodySegment3;
    public Transform bodySegment4;
    OrientationCubeController m_OrientationCube;
    DirectionIndicator m_DirectionIndicator;
    JointDriveController m_JdController;
    private Vector3 m_StartingPos;
    private Vector3 m_moveInput; //Input for movement direction
    private Dictionary<Vector3, float> m_moveInputDict = new Dictionary<Vector3, float>();
    private int currentFrames;
    public override void Initialize()
    {
        m_StartingPos = bodySegment0.position;
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();
        m_JdController = GetComponent<JointDriveController>();

        // カメラの向きに合わせて親オブジェクトを回転
        transform.rotation = Quaternion.Euler(Camera.main.transform.eulerAngles.x, 0, Camera.main.transform.eulerAngles.z);

        UpdateOrientationObjects();

        // 各ボディパーツのセットアップ
        m_JdController.SetupBodyPart(bodySegment0);
        m_JdController.SetupBodyPart(bodySegment1);
        m_JdController.SetupBodyPart(bodySegment2);
        m_JdController.SetupBodyPart(bodySegment3);
        m_JdController.SetupBodyPart(bodySegment4);

        m_moveInputDict.Add(Vector3.forward, 0.25f);
        m_moveInputDict.Add(Vector3.left, 0.25f);
        m_moveInputDict.Add(Vector3.back, 0.25f);
        m_moveInputDict.Add(Vector3.right, 0.25f);
        // m_moveInputDict.Add(Vector3.zero, 0.2f);
    }

    public override void OnEpisodeBegin()
    {
        currentFrames = 0;

        // 各ボディパーツの状態をリセット
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            bodyPart.Reset(bodyPart);
        }

        // 初期回転をランダムに設定（汎化のため）
        bodySegment0.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();
    }
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        // 地面に接しているかどうか
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0); // Whether the bp touching the ground

        // 移動方向基準（ローカル）の速度と角速度を取得
        Quaternion rotation = Quaternion.LookRotation(
            m_moveInput == Vector3.zero ? Vector3.forward : m_moveInput
        );
        Vector3 localVel = Quaternion.Inverse(rotation) * bp.rb.linearVelocity;
        Vector3 localAngVel = Quaternion.Inverse(rotation) * bp.rb.angularVelocity;
        sensor.AddObservation(localVel);
        sensor.AddObservation(localAngVel);

        if (bp.rb.transform != bodySegment0)
        {
            // bodySegment0から見た相対位置（方向基準で変換）
            sensor.AddObservation(
                Quaternion.Inverse(rotation) * (bp.rb.position - bodySegment0.position));
            // 回転情報（ローカル）
            sensor.AddObservation(bp.rb.transform.localRotation);
        }

        // ジョイントの出力強度（正規化）
        if (bp.joint)
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // 地面までの距離（最大10mまでを正規化）
        RaycastHit hit;
        float maxDist = 10;
        if (Physics.Raycast(bodySegment0.position, Vector3.down, out hit, maxDist))
        {
            sensor.AddObservation(hit.distance / maxDist);
        }
        else
            sensor.AddObservation(1); // 地面に当たらなかった場合

        // 目標速度方向をローカルに変換
        var velGoal = m_moveInput.normalized * m_MaxWalkingSpeed;
        sensor.AddObservation(transform.InverseTransformDirection(velGoal));
        // エージェントの向きと移動入力方向との角度（正規化）
        sensor.AddObservation(Vector3.Angle(transform.forward,
                                  m_moveInput.normalized) / 180);
        // bodySegment0の向きと移動方向の角度（0〜180度）
        sensor.AddObservation(Vector3.Angle(bodySegment0.forward, m_moveInput.normalized));

        // 入力方向をローカル空間に変換して観察
        sensor.AddObservation(transform.InverseTransformDirection(m_moveInput));

        // 各ボディパーツの観察を追加
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        Debug.Log($"[ActionReceived] {string.Join(", ", actionBuffers.ContinuousActions.ToArray())}");
        var bpDict = m_JdController.bodyPartsDict;

        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;
        // 各ボディパーツのジョイント回転を設定
        bpDict[bodySegment0].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment2].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment3].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        // ジョイントの強度を設定
        bpDict[bodySegment0].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment1].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment2].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment3].SetJointStrength(continuousActions[++i]);

        // 地面から落ちたらエピソード終了
        if (bodySegment0.position.y < m_StartingPos.y - 4)
        {
            EndEpisode();
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        /*var velReward =
            GetMatchingVelocityReward(m_OrientationCube.transform.forward * m_MaxWalkingSpeed,
                m_JdController.bodyPartsDict[bodySegment0].rb.linearVelocity);

        //Angle of the rotation delta between cube and body.
        //This will range from (0, 180)
        var rotAngle = Quaternion.Angle(m_OrientationCube.transform.rotation,
            m_JdController.bodyPartsDict[bodySegment0].rb.rotation);

        //The reward for facing the target
        var facingRew = 0f;
        //If we are within 60 degrees of facing the target
        if (rotAngle < 60)
        {
            //Set normalized facingReward
            //Facing the target perfectly yields a reward of 1
            facingRew = 1 - (rotAngle / 180);
        }

        //Add the product of these two rewards
        AddReward(velReward * facingRew);*/

        // 蛇の速度と入力方向の一致度（dot）を報酬に反映
        Vector3 agentVel = bodySegment0.GetComponent<Rigidbody>().linearVelocity;
        float dot = Vector3.Dot(agentVel.normalized, m_moveInput.normalized);
        AddReward(dot / 1000f);
        // デバッグ用にdotと速度を表示
        if (dot > 0)
        {
            Debug.Log($"dot: {dot}, vel: {agentVel}, move: {m_moveInput}");
        }
    }
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, m_MaxWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / m_MaxWalkingSpeed, 2), 2);
    }
    void Update()
    {
        // 学習中なら一定フレームごとにランダムな方向を選択
        if (Academy.Instance.IsCommunicatorOn)
        {
            if (currentFrames % 300 == 0)
            {
                m_moveInput = RandomChoiceFromActions(m_moveInputDict);
            }
            currentFrames++;
            Debug.Log($"IsCommunicatorOn Current Frames: {currentFrames}, MoveaInput: {m_moveInput}");
        }
        else
        {
            // 手動操作（矢印キー）
            if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
            {
                m_moveInput = Vector3.forward;
            }
            else if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
            {
                m_moveInput = Vector3.back;
            }
            else if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
            {
                m_moveInput = Vector3.left;
            }
            else if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
            {
                m_moveInput = Vector3.right;
            }
            else
            {
                m_moveInput = Vector3.forward; // デフォルトの移動方向  
            }
            Debug.Log($"Manual MoveInput: {m_moveInput}");
        }
    }
    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(bodySegment0, m_moveInput);

        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }
    Vector3 RandomChoiceFromActions(Dictionary<Vector3, float> actions)
    {
        // 確率付きでランダムな方向を選択
        float tempProbs = 0;
        float randomValue = Random.value;
        foreach (var action in actions)
        {
            tempProbs += action.Value;
            if (randomValue < tempProbs)
            {
                return action.Key;
            }
        }
        return Vector3.zero; // Fallback if no action is chosen
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // ヒューリスティックは0で固定
        var continuousActionsOut = actionsOut.ContinuousActions;

        for (int i = 0; i < continuousActionsOut.Length; i++)
        {
            continuousActionsOut[i] = Random.Range(-10f, 10f);
        }
    }
}
