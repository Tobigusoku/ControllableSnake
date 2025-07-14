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

        UpdateOrientationObjects();

        //Setup each body part
        m_JdController.SetupBodyPart(bodySegment0);
        m_JdController.SetupBodyPart(bodySegment1);
        m_JdController.SetupBodyPart(bodySegment2);
        m_JdController.SetupBodyPart(bodySegment3);
        m_JdController.SetupBodyPart(bodySegment4);

        m_moveInputDict.Add(Vector3.forward, 0.2f);
        m_moveInputDict.Add(Vector3.left, 0.2f);
        m_moveInputDict.Add(Vector3.back, 0.2f);
        m_moveInputDict.Add(Vector3.right, 0.2f);
        m_moveInputDict.Add(Vector3.zero, 0.2f);
    }

    public override void OnEpisodeBegin()
    {
        currentFrames = 0;

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        bodySegment0.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();
    }
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround ? 1 : 0); // Whether the bp touching the ground

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.linearVelocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        if (bp.rb.transform != bodySegment0)
        {
            //Get position relative to hips in the context of our orientation cube's space
            sensor.AddObservation(
                m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - bodySegment0.position));
            sensor.AddObservation(bp.rb.transform.localRotation);
        }

        if (bp.joint)
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        RaycastHit hit;
        float maxDist = 10;
        if (Physics.Raycast(bodySegment0.position, Vector3.down, out hit, maxDist))
        {
            sensor.AddObservation(hit.distance / maxDist);
        }
        else
            sensor.AddObservation(1);

        var cubeForward = m_OrientationCube.transform.forward;
        var velGoal = cubeForward * m_MaxWalkingSpeed;
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.Angle(m_OrientationCube.transform.rotation,
                                  m_JdController.bodyPartsDict[bodySegment0].rb.rotation) / 180);
        sensor.AddObservation(Quaternion.FromToRotation(bodySegment0.forward, cubeForward));

        //Add pos of target relative to orientation cube
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(m_moveInput));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // The dictionary with all the body parts in it are in the jdController
        var bpDict = m_JdController.bodyPartsDict;

        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;
        // Pick a new target joint rotation
        bpDict[bodySegment0].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment1].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment2].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[bodySegment3].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        // Update joint strength
        bpDict[bodySegment0].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment1].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment2].SetJointStrength(continuousActions[++i]);
        bpDict[bodySegment3].SetJointStrength(continuousActions[++i]);

        //Reset if Worm fell through floor;
        if (bodySegment0.position.y < m_StartingPos.y - 2)
        {
            EndEpisode();
        }
    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        var velReward =
            GetMatchingVelocityReward(m_OrientationCube.transform.forward * m_MaxWalkingSpeed,
                m_JdController.bodyPartsDict[bodySegment0].rb.linearVelocity);

        //Angle of the rotation delta between cube and body.
        //This will range from (0, 180)
        var rotAngle = Quaternion.Angle(m_OrientationCube.transform.rotation,
            m_JdController.bodyPartsDict[bodySegment0].rb.rotation);

        //The reward for facing the target
        var facingRew = 0f;
        //If we are within 30 degrees of facing the target
        if (rotAngle < 30)
        {
            //Set normalized facingReward
            //Facing the target perfectly yields a reward of 1
            facingRew = 1 - (rotAngle / 180);
        }

        //Add the product of these two rewards
        AddReward(velReward * facingRew);
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
        if (Academy.Instance.IsCommunicatorOn)
        {
            currentFrames++;
            if (currentFrames % 300 == 0)
            {
                m_moveInput = RandomChoiceFromActions(m_moveInputDict);
            }
        }
        else
        {
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
                m_moveInput = Vector3.zero;
            }
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
        var continuousActionsOut = actionsOut.ContinuousActions;

        for (int i = 0; i < continuousActionsOut.Length; i++)
        {
            continuousActionsOut[i] = 0f;
        }
    }
}
