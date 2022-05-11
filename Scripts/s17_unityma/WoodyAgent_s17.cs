using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

using System.Collections;
using System.Reflection;
public class WoodyAgent_s17 : Agent
{
    //------------------------------------------------------------------------------------------
    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
    EnvController_s17 woodyarea;
    Transform board, ball, ball_target_tmark;
    GameObject support;
    // Rigidbody boardrb, ballrb;
    Rigidbody upper_arm_right, lower_arm_right, hand_right, upper_arm_left, lower_arm_left, hand_left;
    Transform hand_right_end, hand_left_end;
    Transform upper_arm_right_joint, lower_arm_right_joint, hand_right_joint, upper_arm_left_joint, lower_arm_left_joint, hand_left_joint;

    int action_count;
    public Transform l1, r1; // the three holding points on the board
    [HideInInspector]
    public Transform al1, ar1;// the three holding points on the agent hands
    // use to calculate rewards
    Vector3 board_target; // the target position where the board supposed to stay at balance status
    // use to access parameters from outside of C# and Unity
    float force_magnitude = 1.0f;  // the force magnitude adds to each joint
    Vector3 upper_arm_right_torque, lower_arm_right_torque, hand_right_torque, upper_arm_left_torque, lower_arm_left_torque, hand_left_torque;

    float observe_index = 1f;

    float action_index = 9f; // from action_index=3


    // string log = ""; // use to hold the log information for check in standalone exe

    // float time = 0f;
    float handreach_reward = 0f; // the following three rewards is to adding them into the observation as extra 3 dimensions, to pass them into the python API for CRPO approach
    float boardreach_reward = 0f;
    float ballreach_reward = 0f;

    public int action_step=0;
    public int obs_step=0;

    
    public override void Initialize()
    {
        woodyarea = GameObject.Find("WoodyArea").GetComponent<EnvController_s17>();
        board = woodyarea.board;
        // target = woodyarea.target;
        ball = woodyarea.ball;
        support = woodyarea.support;

        // boardrb = woodyarea.boardrb;
        // ballrb = woodyarea.ballrb;

        ball_target_tmark = woodyarea.ball_target_tmark;
        board_target = woodyarea.board_target;
        // get the reference of all needed joints
        GetJointsRef();
        // initialize the pre values for distance or angle calculation for reward
        al1 = FindChildObjectwithName(this.transform, "l1");

        ar1 = FindChildObjectwithName(this.transform, "r1");

        action_count = this.transform.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions;
    }

    // get the joint reference of the character for the later on calculation
    public void GetJointsRef()
    {
        upper_arm_right = FindChildObjectwithName(this.transform, "upper_arm_right").GetComponent<Rigidbody>();
        lower_arm_right = FindChildObjectwithName(this.transform, "lower_arm_right").GetComponent<Rigidbody>();
        hand_right = FindChildObjectwithName(this.transform, "hand_right").GetComponent<Rigidbody>();
        hand_right_end = FindChildObjectwithName(this.transform, "hand_right_end");
        upper_arm_left = FindChildObjectwithName(this.transform, "upper_arm_left").GetComponent<Rigidbody>();
        lower_arm_left = FindChildObjectwithName(this.transform, "lower_arm_left").GetComponent<Rigidbody>();
        hand_left = FindChildObjectwithName(this.transform, "hand_left").GetComponent<Rigidbody>();
        hand_left_end = FindChildObjectwithName(this.transform, "hand_left_end");

        upper_arm_right_joint = FindChildObjectwithName(this.transform, "upper_arm_right_joint");
        lower_arm_right_joint = FindChildObjectwithName(this.transform, "lower_arm_right_joint");
        hand_right_joint = FindChildObjectwithName(this.transform, "hand_right_joint");
        upper_arm_left_joint = FindChildObjectwithName(this.transform, "upper_arm_left_joint");
        lower_arm_left_joint = FindChildObjectwithName(this.transform, "lower_arm_left_joint");
        hand_left_joint = FindChildObjectwithName(this.transform, "hand_left_joint");
    }

    public override void CollectObservations(VectorSensor sensor) // 123 + 3 = 126
    {
        var methodName = "Observation" + observe_index.ToString();

        if(observe_index > 0) SendMessage(methodName, sensor);

        else Debug.Log("There is no observation given!");

        obs_step += 1;
       
    //    Debug.Log(this.name + " ------- observation step " + obs_step);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        
        
        ActionSegment<float> vectoraction = actionBuffers.ContinuousActions;

        
        
        var methodName = "TakeAction" + action_index.ToString();

        if(action_index >= 0) SendMessage(methodName, vectoraction);

        else Debug.Log("There is no action received!");

        // Debug.Log(" ball mass= " + ball.GetComponent<Rigidbody>().mass + ", board mass= " + board.GetComponent<Rigidbody>().mass);
        action_step += 1;
    }

    void TakeAction9(ActionSegment<float> vectoraction) // use only lower arm and hands, action dim=4, only torque used
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 10f;  //50f
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        Debug.Log("-----------------TakeAction9-------------------");
        lower_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1]/10f, vectoraction[2]);
        hand_right_torque = new Vector3(0f, 0f, vectoraction[3]/10f);

        lower_arm_left_torque = new Vector3(vectoraction[4], vectoraction[5]/10f, vectoraction[6]);
        hand_left_torque = new Vector3(0f, 0f, vectoraction[7]/10f);

        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

        // var hand_right_force = new Vector3(0f, vectoraction[8], 0f);
        // var hand_left_force = new Vector3(0f, vectoraction[9], 0f);

        var hand_right_force = new Vector3(vectoraction[8], 0f, 0f);
        var hand_left_force = new Vector3(vectoraction[9], 0f, 0f);
        // hand_right.AddForceAtPosition(hand_right_force, r1.position, ForceMode.Force);
        // hand_left.AddForceAtPosition(hand_left_force, l1.position, ForceMode.Force); //, limbends["hand_left_end"].position);
        hand_right.AddRelativeForce(hand_right_force, ForceMode.Force);
        hand_left.AddRelativeForce(hand_left_force, ForceMode.Force);

    }
    void TakeAction8(ActionSegment<float> vectoraction) // use only lower arm and hands, action dim=4, only torque used
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 10f;  //50f
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        Debug.Log("-----------------TakeAction8-------------------");
        lower_arm_right_torque = new Vector3(0f, vectoraction[0]/10f, vectoraction[1]);
        hand_right_torque = new Vector3(vectoraction[2], 0f, vectoraction[3]);

        lower_arm_left_torque = new Vector3(0f, vectoraction[4]/10f, vectoraction[5]);
        hand_left_torque = new Vector3(vectoraction[6], 0f, vectoraction[7]);

        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }
    void TakeAction7(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 10f;  //50f
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        Debug.Log( " TakeAction0 " + this.name + ", " + action_step + " action is: " + vectoraction[0].ToString("F3") + ", " + vectoraction[1].ToString("F3") + 
            ", " + vectoraction[2].ToString("F3") + ", " + vectoraction[3].ToString("F3") + ", " + vectoraction[4].ToString("F3") + ", " + 
            vectoraction[5].ToString("F3") + ", " + vectoraction[6].ToString("F3") + ", " + vectoraction[7].ToString("F3") + ", " + vectoraction[8].ToString("F3") +
            vectoraction[9].ToString("F3") + ", " + vectoraction[10].ToString("F3") + ", " + vectoraction[11].ToString("F3"));

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1]/10f, vectoraction[2]);
        lower_arm_right_torque = new Vector3(0f, vectoraction[3]/10f, vectoraction[4]);
        hand_right_torque = new Vector3(vectoraction[5], 0f, vectoraction[6]);

        upper_arm_left_torque = new Vector3(vectoraction[7], vectoraction[8]/10f, vectoraction[9]);
        lower_arm_left_torque = new Vector3(0f, vectoraction[10]/10f, vectoraction[11]);
        hand_left_torque = new Vector3(vectoraction[12], 0f, vectoraction[13]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }
    void TakeAction0(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 5f; // 50f with tray on it 5f without tray on it.
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        Debug.Log( " TakeAction0 " + this.name + ", " + action_step + " action is: " + vectoraction[0].ToString("F3") + ", " + vectoraction[1].ToString("F3") + 
            ", " + vectoraction[2].ToString("F3") + ", " + vectoraction[3].ToString("F3") + ", " + vectoraction[4].ToString("F3") + ", " + 
            vectoraction[5].ToString("F3") + ", " + vectoraction[6].ToString("F3") + ", " + vectoraction[7].ToString("F3") + ", " + vectoraction[8].ToString("F3") +
            vectoraction[9].ToString("F3") + ", " + vectoraction[10].ToString("F3") + ", " + vectoraction[11].ToString("F3"));

        upper_arm_right_torque = new Vector3(vectoraction[0], 0f, vectoraction[1]);
        lower_arm_right_torque = new Vector3(0f, vectoraction[2]/10f, vectoraction[3]);
        hand_right_torque = new Vector3(vectoraction[4], 0f, vectoraction[5]);

        upper_arm_left_torque = new Vector3(vectoraction[6], 0f, vectoraction[7]);
        lower_arm_left_torque = new Vector3(0f, vectoraction[8]/10f, vectoraction[9]);
        hand_left_torque = new Vector3(vectoraction[10], 0f, vectoraction[11]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

        // ForceMode.torque
    }

    void TakeAction1(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 10f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1], vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4], vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7], vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10], vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13], vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16], vectoraction[17]);

        upper_arm_right.AddRelativeForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeForce(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeForce(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeForce(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeForce(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeForce(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }

    void TakeAction2(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 100f;
            // vectoraction[i] = Mathf.Clamp(Random.Range(-1f, 1f), -1f, 1f) * force_magnitude * 100f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        // Debug.Log("-------------action received: " + vectoraction[0] +", " + vectoraction[1] + ", " + vectoraction[2] );
        
        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1], vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4], vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7], vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10], vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13], vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16], vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }

    void TakeAction3(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*5f;
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        Debug.Log( this.name + ", " + action_step + " action is: " + vectoraction[0] + ", " + vectoraction[1] + ", " + vectoraction[2]);

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1]/10f, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4]/10f, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7]/100f, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10]/10f, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13]/10f, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16]/10f, vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

        // ForceMode.torque
    }

    void TakeAction4(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*10f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        upper_arm_right_torque = new Vector3(vectoraction[0], 0, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], 0, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], 0, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], 0, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], 0, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], 0, vectoraction[17]);

        upper_arm_right.AddForceAtPosition(upper_arm_right_torque, lower_arm_right_joint.position, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddForceAtPosition(lower_arm_right_torque, hand_right_joint.position, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddForceAtPosition(hand_right_torque, r1.position, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddForceAtPosition(upper_arm_left_torque, lower_arm_left_joint.position, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddForceAtPosition(lower_arm_left_torque, hand_left_joint.position, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddForceAtPosition(hand_left_torque, l1.position, ForceMode.Force); //, limbends["hand_left_end"].position);

        upper_arm_right.AddRelativeTorque(new Vector3(0, vectoraction[1]/100f, 0),ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(new Vector3(0, vectoraction[4]/100f, 0), ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(new Vector3(0, vectoraction[7]/100f, 0), ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(new Vector3(0, vectoraction[10]/100f, 0), ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(new Vector3(0, vectoraction[13]/100f, 0), ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(new Vector3(0, vectoraction[16]/100f, 0), ForceMode.Force); //, limbends["hand_left_end"].position);

    }

    void TakeAction5(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*100f;
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        if(action_step % 5 ==0)
        {
            Vector3 lpre_upper_arm, lpre_lower_arm, lpre_hand, rpre_upper_arm, rpre_lower_arm, rpre_hand;
            int agent_index = 0;
            if(this.name == "Agent1") agent_index = 0;
            if(this.name == "Agent2") agent_index = 1;

            lpre_upper_arm = woodyarea.AgentsList[agent_index].lpre_upper_arm;
            lpre_lower_arm = woodyarea.AgentsList[agent_index].lpre_lower_arm;
            lpre_hand = woodyarea.AgentsList[agent_index].lpre_hand;

            rpre_upper_arm = woodyarea.AgentsList[agent_index].rpre_upper_arm;
            rpre_lower_arm = woodyarea.AgentsList[agent_index].rpre_lower_arm;
            rpre_hand = woodyarea.AgentsList[agent_index].rpre_hand;

            Debug.Log( this.name + ", " + action_step + " action is: " + vectoraction[0] + ", " + vectoraction[1] + ", " + vectoraction[2]);

            upper_arm_right_torque = rpre_upper_arm +  new Vector3(vectoraction[0], vectoraction[1]/100f, vectoraction[2]);
            lower_arm_right_torque = rpre_lower_arm + new Vector3(vectoraction[3], vectoraction[4]/100f, vectoraction[5]);
            hand_right_torque = rpre_hand + new Vector3(vectoraction[6], vectoraction[7]/100f, vectoraction[8]);

            upper_arm_left_torque = lpre_upper_arm + new Vector3(vectoraction[9], vectoraction[10]/100f, vectoraction[11]);
            lower_arm_left_torque =lpre_lower_arm + new Vector3(vectoraction[12], vectoraction[13]/100f, vectoraction[14]);
            hand_left_torque = lpre_hand + new Vector3(vectoraction[15], vectoraction[16]/100f, vectoraction[17]);

            upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
            hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

            upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
            hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

            // ForceMode.torque

            woodyarea.AgentsList[agent_index].lpre_upper_arm = upper_arm_left_torque;
            woodyarea.AgentsList[agent_index].lpre_lower_arm = lower_arm_left_torque;
            woodyarea.AgentsList[agent_index].lpre_hand = hand_left_torque;

            woodyarea.AgentsList[agent_index].rpre_upper_arm = upper_arm_right_torque;
            woodyarea.AgentsList[agent_index].rpre_lower_arm = lower_arm_right_torque;
            woodyarea.AgentsList[agent_index].rpre_hand = hand_right_torque;
        }
        else
        {
            upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
            hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

            upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
            hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
        }

        Debug.Log( this.name + ",upper arm torque is: " + upper_arm_left_torque + ", " + upper_arm_right_torque);

    }

    void TakeAction6(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*1f;
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        if(action_step % 5 ==0)
        {
            Vector3 lpre_upper_arm, lpre_lower_arm, lpre_hand, rpre_upper_arm, rpre_lower_arm, rpre_hand;
            int agent_index = 0;
            if(this.name == "Agent1") agent_index = 0;
            if(this.name == "Agent2") agent_index = 1;

            lpre_upper_arm = woodyarea.AgentsList[agent_index].lpre_upper_arm;
            lpre_lower_arm = woodyarea.AgentsList[agent_index].lpre_lower_arm;
            lpre_hand = woodyarea.AgentsList[agent_index].lpre_hand;

            rpre_upper_arm = woodyarea.AgentsList[agent_index].rpre_upper_arm;
            rpre_lower_arm = woodyarea.AgentsList[agent_index].rpre_lower_arm;
            rpre_hand = woodyarea.AgentsList[agent_index].rpre_hand;

            Debug.Log( this.name + ", " + action_step + " action is: " + vectoraction[0] + ", " + vectoraction[1] + ", " + vectoraction[2]);

            upper_arm_right_torque = new Vector3(vectoraction[0], 0f, vectoraction[1]);
            lower_arm_right_torque = new Vector3(0f, vectoraction[2]/10f, vectoraction[3]);
            hand_right_torque = new Vector3(vectoraction[4], 0f, vectoraction[5]);

            upper_arm_left_torque = new Vector3(vectoraction[6], 0f, vectoraction[7]);
            lower_arm_left_torque = new Vector3(0f, vectoraction[8]/10f, vectoraction[9]);
            hand_left_torque = new Vector3(vectoraction[10], 0f, vectoraction[11]);

            upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
            hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

            upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
            hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

            // ForceMode.torque

            woodyarea.AgentsList[agent_index].lpre_upper_arm = upper_arm_left_torque;
            woodyarea.AgentsList[agent_index].lpre_lower_arm = lower_arm_left_torque;
            woodyarea.AgentsList[agent_index].lpre_hand = hand_left_torque;

            woodyarea.AgentsList[agent_index].rpre_upper_arm = upper_arm_right_torque;
            woodyarea.AgentsList[agent_index].rpre_lower_arm = lower_arm_right_torque;
            woodyarea.AgentsList[agent_index].rpre_hand = hand_right_torque;
        }
        else
        {
            upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
            hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

            upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
            hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
        }

        Debug.Log( this.name + ",upper arm torque is: " + upper_arm_left_torque + ", " + upper_arm_right_torque);

    }

    void Observation1(VectorSensor sensor) //139-16 = 123
    {
        //3
        sensor.AddObservation(ball_target_tmark.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(woodyarea.ballrb.velocity);
        sensor.AddObservation(woodyarea.ballrb.angularVelocity / woodyarea.ballrb.maxAngularVelocity);

        sensor.AddObservation(ball.position - ball_target_tmark.position);
        sensor.AddObservation(Vector3.Distance(ball.position, ball_target_tmark.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(woodyarea.boardrb.velocity);
        sensor.AddObservation(woodyarea.boardrb.angularVelocity / woodyarea.boardrb.maxAngularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity / upper_arm_left.maxAngularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity / lower_arm_left.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_left.velocity);
        
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity / hand_left.maxAngularVelocity);
        sensor.AddObservation(hand_left.velocity);
        
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity / upper_arm_right.maxAngularVelocity); 
        sensor.AddObservation(upper_arm_right.velocity); 
        
        sensor.AddObservation(lower_arm_right.transform.position); 
        sensor.AddObservation(lower_arm_right.transform.rotation); 
        sensor.AddObservation(lower_arm_right.angularVelocity/lower_arm_right.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_right.velocity); 
        
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity / hand_right.maxAngularVelocity);
        sensor.AddObservation(hand_right.velocity);
    }
    
    void Observation2(VectorSensor sensor) //139 - 16 = 123
    {
        //3
        sensor.AddObservation(ball_target_tmark.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(woodyarea.ballrb.velocity);
        sensor.AddObservation(woodyarea.ballrb.angularVelocity / woodyarea.ballrb.maxAngularVelocity);

        sensor.AddObservation(ball.position - ball_target_tmark.position);
        sensor.AddObservation(Vector3.Distance(ball.position, ball_target_tmark.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(woodyarea.boardrb.velocity);
        sensor.AddObservation(woodyarea.boardrb.angularVelocity / woodyarea.boardrb.maxAngularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.localPosition);
        sensor.AddObservation(upper_arm_left.transform.localRotation);
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.angularVelocity));
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.velocity));
        
        sensor.AddObservation(lower_arm_left.transform.localPosition);
        sensor.AddObservation(lower_arm_left.transform.localRotation);
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.angularVelocity)); 
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.velocity));
        
        sensor.AddObservation(hand_left.transform.localPosition);
        sensor.AddObservation(hand_left.transform.localRotation);
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.angularVelocity));
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.velocity));
        
        sensor.AddObservation(upper_arm_right.transform.localPosition);
        sensor.AddObservation(upper_arm_right.transform.localRotation);
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.angularVelocity)); 
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.velocity)); 
        
        sensor.AddObservation(lower_arm_right.transform.localPosition); 
        sensor.AddObservation(lower_arm_right.transform.localRotation); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.angularVelocity)); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.velocity)); 
        
        sensor.AddObservation(hand_right.transform.localPosition);
        sensor.AddObservation(hand_right.transform.localRotation);
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.angularVelocity));
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.velocity));

        // Debug.Log(this.name + " hand position " + hand_right.transform.position.ToString("F3") + " local position " + hand_right.transform.localPosition.ToString("F3") + " inversedirection ");
        // Debug.Log(this.name + " hand local "+ hand_right.transform.InverseTransformPoint(hand_right.transform.position).ToString("F3"));
        // Debug.Log(this.name + " lower arm local" + lower_arm_right_joint.transform.InverseTransformPoint(hand_right.transform.position).ToString("F3"));
        // Debug.Log(this.name + " angularv " + hand_right.angularVelocity + " local av " + hand_right.transform.InverseTransformPoint(hand_right.angularVelocity));
        // Debug.Log(this.name + "hand-joint " + (hand_right.transform.position - hand_right_joint.transform.position).ToString("F3"));
        // Debug.Log(this.name + "hand-lowerarm " + (hand_right.transform.position - lower_arm_right.transform.position).ToString("F3"));


    }

    void Observation3(VectorSensor sensor) //139 - 16 = 123 + 3 = 126
    {
        //3
        sensor.AddObservation(ball_target_tmark.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(woodyarea.ballrb.velocity);
        sensor.AddObservation(woodyarea.ballrb.angularVelocity / woodyarea.ballrb.maxAngularVelocity);

        sensor.AddObservation(ball.position - ball_target_tmark.position);
        sensor.AddObservation(Vector3.Distance(ball.position, ball_target_tmark.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(woodyarea.boardrb.velocity);
        sensor.AddObservation(woodyarea.boardrb.angularVelocity / woodyarea.boardrb.maxAngularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity / upper_arm_left.maxAngularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity / lower_arm_left.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_left.velocity);
        
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity / hand_left.maxAngularVelocity);
        sensor.AddObservation(hand_left.velocity);
        
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity / upper_arm_right.maxAngularVelocity); 
        sensor.AddObservation(upper_arm_right.velocity); 
        
        sensor.AddObservation(lower_arm_right.transform.position); 
        sensor.AddObservation(lower_arm_right.transform.rotation); 
        sensor.AddObservation(lower_arm_right.angularVelocity / lower_arm_right.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_right.velocity); 
        
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity / hand_right.maxAngularVelocity);
        sensor.AddObservation(hand_right.velocity);

        sensor.AddObservation(handreach_reward);
        sensor.AddObservation(boardreach_reward);
        sensor.AddObservation(ballreach_reward);

    }

    void Observation4(VectorSensor sensor) //139 - 16 = 123 + 3 = 126
    {
        //3
        sensor.AddObservation(ball_target_tmark.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(woodyarea.ballrb.velocity);
        sensor.AddObservation(woodyarea.ballrb.angularVelocity);

        sensor.AddObservation(ball.position - ball_target_tmark.position);
        sensor.AddObservation(Vector3.Distance(ball.position, ball_target_tmark.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(woodyarea.boardrb.velocity);
        sensor.AddObservation(woodyarea.boardrb.angularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.localPosition);
        sensor.AddObservation(upper_arm_left.transform.localRotation);
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.angularVelocity));
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.velocity));
        
        sensor.AddObservation(lower_arm_left.transform.localPosition);
        sensor.AddObservation(lower_arm_left.transform.localRotation);
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.angularVelocity)); 
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.velocity));
        
        sensor.AddObservation(hand_left.transform.localPosition);
        sensor.AddObservation(hand_left.transform.localRotation);
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.angularVelocity));
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.velocity));
        
        sensor.AddObservation(upper_arm_right.transform.localPosition);
        sensor.AddObservation(upper_arm_right.transform.localRotation);
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.angularVelocity)); 
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.velocity)); 
        
        sensor.AddObservation(lower_arm_right.transform.localPosition); 
        sensor.AddObservation(lower_arm_right.transform.localRotation); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.angularVelocity)); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.velocity)); 
        
        sensor.AddObservation(hand_right.transform.localPosition);
        sensor.AddObservation(hand_right.transform.localRotation);
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.angularVelocity));
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.velocity));

        sensor.AddObservation(handreach_reward);
        sensor.AddObservation(boardreach_reward);
        sensor.AddObservation(ballreach_reward);

    }

    public Transform FindChildObjectwithName(Transform parent, string name)
    {
        for (var i = 0; i < parent.childCount; i++)
        {
            var child = parent.GetChild(i);
            if (child.name == name)
            {
                return child;
            }
            if (child.childCount > 0)
            {
                var found = FindChildObjectwithName(child, name);
                if (found != null)
                {
                    return found;
                }
            }
        }
        return null;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

        float torqueforce = force_magnitude;

        ActionSegment<float> actions = actionsOut.ContinuousActions;

        // Random.Range(-1.0f,1.0f);

        // actions[0] = Random.Range(-1.0f, 1.0f);
        // actions[1] = Random.Range(-1.0f, 1.0f);
        // actions[2] = Random.Range(-1.0f, 1.0f);
        // actions[3] = Random.Range(-1.0f, 1.0f);
        // actions[4] = Random.Range(-1.0f, 1.0f);
        // actions[5] = Random.Range(-1.0f, 1.0f);
        // actions[6] = Random.Range(-1.0f, 1.0f);
        // actions[7] = Random.Range(-1.0f, 1.0f);
        // actions[8] = Random.Range(-1.0f, 1.0f);
        // actions[9] = Random.Range(-1.0f, 1.0f);
        // actions[10]= Random.Range(-1.0f, 1.0f);
        // actions[11]= Random.Range(-1.0f, 1.0f);
        // actions[12]= Random.Range(-1.0f, 1.0f);
        // actions[13]= Random.Range(-1.0f, 1.0f);

        // actions[0] = 0f;
        // actions[1] = 0f;
        // actions[2] = 0f;
        // actions[3] = 0f;
        // actions[4] = 0f;
        // actions[5] = 0f;
        // actions[6] =0f;
        // actions[7]=0f;
        // actions[8]=0f;
        // actions[9]=0f;
        // actions[10]=0f;
        // actions[11]=0f;
        // actions[12]=0f;
        // actions[13]=0f;
        // actions[14]=0f;
        // actions[15]=0f;
        // actions[16]=0f;
        // actions[17]=0f;

        if (Input.GetKey(KeyCode.Q)) actions[0] = 1;
        if (Input.GetKey(KeyCode.A)) actions[0] = -1;
        if (Input.GetKey(KeyCode.W)) actions[1] = 1;
        if (Input.GetKey(KeyCode.S)) actions[1] = -1;
        if (Input.GetKey(KeyCode.E)) actions[2] = 1;
        if (Input.GetKey(KeyCode.D)) actions[2] = -1;
        if (Input.GetKey(KeyCode.R)) actions[3] = 1;
        if (Input.GetKey(KeyCode.F)) actions[3] = -1;
        if (Input.GetKey(KeyCode.T)) actions[4] = 1;
        if (Input.GetKey(KeyCode.G)) actions[4] = -1;
        if (Input.GetKey(KeyCode.Y)) actions[5] = 1;
        if (Input.GetKey(KeyCode.H)) actions[5] = -1;
        if (Input.GetKey(KeyCode.U)) actions[6] = 1;
        if (Input.GetKey(KeyCode.J)) actions[6] = -1;

        if(Input.GetKey(KeyCode.I)) actions[7] =  1;
        if(Input.GetKey(KeyCode.K)) actions[7] = -1;
        if(Input.GetKey(KeyCode.O)) actions[9] =  1;
        if(Input.GetKey(KeyCode.L)) actions[9] = -1;

        // if (Input.GetKey(KeyCode.Q)) actions[7] = 1;
        // if (Input.GetKey(KeyCode.A)) actions[7] = -1;
        // if (Input.GetKey(KeyCode.W)) actions[8] = 1;
        // if (Input.GetKey(KeyCode.S)) actions[8] = -1;
        // if (Input.GetKey(KeyCode.E)) actions[9] = 1;
        // if (Input.GetKey(KeyCode.D)) actions[9] = -1;
        // if (Input.GetKey(KeyCode.R)) actions[10] = 1;
        // if (Input.GetKey(KeyCode.F)) actions[10] = -1;
        // if (Input.GetKey(KeyCode.T)) actions[11] = 1;
        // if (Input.GetKey(KeyCode.G)) actions[11] = -1;
        // if (Input.GetKey(KeyCode.Y)) actions[12] = 1;
        // if (Input.GetKey(KeyCode.H)) actions[12] = -1;
        // if (Input.GetKey(KeyCode.U)) actions[13] = 1;
        // if (Input.GetKey(KeyCode.J)) actions[13] = -1;


        // if (Input.GetKey(KeyCode.Y)) actions[14] = 1;
        // if (Input.GetKey(KeyCode.H)) actions[14] = -1;
        // if (Input.GetKey(KeyCode.U)) actions[15] = 1;
        // if (Input.GetKey(KeyCode.J)) actions[15] = -1;
        // if (Input.GetKey(KeyCode.I)) actions[16] = 1;
        // if (Input.GetKey(KeyCode.K)) actions[16] = -1;
        // if (Input.GetKey(KeyCode.O)) actions[17] = 1;
        // if (Input.GetKey(KeyCode.L)) actions[17] = -1;
        
        TestActionFun(actions);
    }

    // public override void Heuristic_d6(in ActionBuffers actionsOut)
    // {

    //     float torqueforce = force_magnitude;

    //     ActionSegment<float> actions = actionsOut.ContinuousActions;

    //     // Random.Range(-1.0f,1.0f);

    //     // actions[0] = Random.Range(-1.0f, 1.0f);
    //     // actions[1] = Random.Range(-1.0f, 1.0f);
    //     // actions[2] = Random.Range(-1.0f, 1.0f);
    //     // actions[3] = Random.Range(-1.0f, 1.0f);
    //     // actions[4] = Random.Range(-1.0f, 1.0f);
    //     // actions[5] = Random.Range(-1.0f, 1.0f);
    //     // actions[6] = Random.Range(-1.0f, 1.0f);
    //     // actions[7] = Random.Range(-1.0f, 1.0f);
    //     // actions[8] = Random.Range(-1.0f, 1.0f);
    //     // actions[9] = Random.Range(-1.0f, 1.0f);
    //     // actions[10]= Random.Range(-1.0f, 1.0f);
    //     // actions[11]= Random.Range(-1.0f, 1.0f);
    //     // actions[12]= Random.Range(-1.0f, 1.0f);
    //     // actions[13]= Random.Range(-1.0f, 1.0f);

    //     // actions[0] = 0f;
    //     // actions[1] = 0f;
    //     // actions[2] = 0f;
    //     // actions[3] = 0f;
    //     // actions[4] = 0f;
    //     // actions[5] = 0f;
    //     // actions[6] =0f;
    //     // actions[7]=0f;
    //     // actions[8]=0f;
    //     // actions[9]=0f;
    //     // actions[10]=0f;
    //     // actions[11]=0f;
    //     // actions[12]=0f;
    //     // actions[13]=0f;
    //     // actions[14]=0f;
    //     // actions[15]=0f;
    //     // actions[16]=0f;
    //     // actions[17]=0f;

    //     // if (Input.GetKey(KeyCode.Q)) actions[0] = 1;
    //     // if (Input.GetKey(KeyCode.A)) actions[0] = -1;
    //     // if (Input.GetKey(KeyCode.W)) actions[1] = 1;
    //     // if (Input.GetKey(KeyCode.S)) actions[1] = -1;
    //     // if (Input.GetKey(KeyCode.E)) actions[2] = 1;
    //     // if (Input.GetKey(KeyCode.D)) actions[2] = -1;
    //     // if (Input.GetKey(KeyCode.R)) actions[3] = 1;
    //     // if (Input.GetKey(KeyCode.F)) actions[3] = -1;
    //     // if (Input.GetKey(KeyCode.T)) actions[4] = 1;
    //     // if (Input.GetKey(KeyCode.G)) actions[4] = -1;
    //     // if (Input.GetKey(KeyCode.Y)) actions[5] = 1;
    //     // if (Input.GetKey(KeyCode.H)) actions[5] = -1;

    //     // if (Input.GetKey(KeyCode.Q)) actions[6] = 1;
    //     // if (Input.GetKey(KeyCode.A)) actions[6] = -1;
    //     // if (Input.GetKey(KeyCode.W)) actions[7] = 1;
    //     // if (Input.GetKey(KeyCode.S)) actions[7] = -1;
    //     // if (Input.GetKey(KeyCode.E)) actions[8] = 1;
    //     // if (Input.GetKey(KeyCode.D)) actions[8] = -1;

    //     // if (Input.GetKey(KeyCode.R)) actions[9] = 1;
    //     // if (Input.GetKey(KeyCode.F)) actions[9] = -1;
    //     // if (Input.GetKey(KeyCode.T)) actions[10] = 1;
    //     // if (Input.GetKey(KeyCode.G)) actions[10] = -1;
    //     // if (Input.GetKey(KeyCode.Y)) actions[11] = 1;
    //     // if (Input.GetKey(KeyCode.H)) actions[11] = -1;


    //     // if (Input.GetKey(KeyCode.U)) actions[12] = 1;
    //     // if (Input.GetKey(KeyCode.J)) actions[12] = -1;
    //     // if (Input.GetKey(KeyCode.T)) actions[13] = 1;
    //     // if (Input.GetKey(KeyCode.G)) actions[13] = -1;
    //     // if (Input.GetKey(KeyCode.Y)) actions[14] = 1;
    //     // if (Input.GetKey(KeyCode.H)) actions[14] = -1;
    //     // if (Input.GetKey(KeyCode.U)) actions[15] = 1;
    //     // if (Input.GetKey(KeyCode.J)) actions[15] = -1;
    //     // if (Input.GetKey(KeyCode.I)) actions[16] = 1;
    //     // if (Input.GetKey(KeyCode.K)) actions[16] = -1;
    //     // if (Input.GetKey(KeyCode.O)) actions[17] = 1;
    //     // if (Input.GetKey(KeyCode.L)) actions[17] = -1;
        
    //     TestActionFun(actions);
    // }
    public void TestActionFun(ActionSegment<float> action)
    {
       if (action[0] > 0) Debug.Log(this.name + " Q pressed, lefthold X joint force increased" + " " + action[0] + " " + l1.position.ToString("F3"));
       if (action[0] < 0) Debug.Log(this.name + " A pressed, lefthold X joint force decreased" + " " + action[0] + " " + l1.position.ToString("F3"));
       if (action[1] > 0) Debug.Log(this.name + " W pressed, lefthold Y joint force increased" + " " + action[1] + " " + l1.position.ToString("F3"));
       if (action[1] < 0) Debug.Log(this.name + " S pressed, lefthold Y joint force decreased" + " " + action[1] + " " + l1.position.ToString("F3"));
       if (action[2] > 0) Debug.Log(this.name + " E pressed, lefthold Z joint force increased" + " " + action[2] + " " + l1.position.ToString("F3"));
       if (action[2] < 0) Debug.Log(this.name + " D pressed, lefthold Z joint force decreased" + " " + action[2] + " " + l1.position.ToString("F3"));
       if (action[3] > 0) Debug.Log(this.name + " R pressed, righthold X joint force increased" + " " + action[3] + " " + r1.position.ToString("F3"));
       if (action[3] < 0) Debug.Log(this.name + " F pressed, righthold X joint force decreased" + " " + action[3] + " " + r1.position.ToString("F3"));
       if (action[4] > 0) Debug.Log(this.name + " T pressed, righthold Y joint force increased" + " " + action[4] + " " + r1.position.ToString("F3"));
       if (action[4] < 0) Debug.Log(this.name + " G pressed, righthold Y joint force decreased" + " " + action[4] + " " + r1.position.ToString("F3"));
       if (action[5] > 0) Debug.Log(this.name + " Y pressed, righthold Z joint force increased" + " " + action[5] + " " + r1.position.ToString("F3"));
       if (action[5] < 0) Debug.Log(this.name + " H pressed, righthold Z joint force decreased" + " " + action[5] + " " + r1.position.ToString("F3"));
       
    }
}


