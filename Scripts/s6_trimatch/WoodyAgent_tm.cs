using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class WoodyAgent_tm : Agent
{
    //------------------------------------------------------------------------------------------
    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
    //public bool invertagent; // this bool value will help to differenciate the two agent

    //GameObject agent; // source prefab and the generated agent

    WoodyArea woodyarea;
    Transform board, target, ball;
    Rigidbody boardrigid, ballrigid;

    Rigidbody upper_arm_right;
    Rigidbody lower_arm_right;
    Rigidbody hand_right;
    Transform hand_right_end;
    Rigidbody upper_arm_left;
    Rigidbody lower_arm_left;
    Rigidbody hand_left;
    Transform hand_left_end;

    public Transform l1, l2, l3, r1, r2, r3; // the three holding points on the board
    Transform al1, al2, al3, ar1, ar2, ar3;// the three holding points on the agent hands

    float cur_l1, cur_l2, cur_l3, cur_r1, cur_r2, cur_r3; // the current distance of holding point between hand and board
    float pre_l1, pre_l2, pre_l3, pre_r1, pre_r2, pre_r3;

    int action_count = 18;

    Material crosswin, crosslose;

    //private Vector3 agentfacedir; // the facing directioin of the current agent
    public Transform lefthold, righthold; // use to set the target hold location for both hands
    CharacterJoint leftholdjoint, rightholdjoint; // use to store the joint on the holder

    [HideInInspector]
    public int countreach = 0; // use to check the success terminate condition
    int countdist = 0;

    // use to calculate rewards
    float pre_leftdist, pre_rightdist, cur_leftdist, cur_rightdist; // to calculate reward for hands position
    float pre_leftang, pre_rightang, cur_leftang, cur_rightang, cur_leftrotup, cur_rightrotup; // to calculate reward for palm rotation
    float pre_balldist, cur_balldist; // to award the reward when the ball arrives to the target location

    float pre_boarddist_y, cur_boarddist_y, pre_boardang, cur_boardang; // use to calculate reward for board balance in position and angle
    float board_target_y; // the target position where the board supposed to stay at balance status

    RaycastHit lefthit, righthit; // use to detect if the hand palm facing the board

    // use to access parameters from outside of C# and Unity
    EnvironmentParameters m_ResetParams;
    float force_magnitude = 0.0f;  // the force magnitude adds to each joint
    float terminate_steps = 0;       // the max steps needs for each episode, use to calculate the time penalty

    float target_distance = 0.02f;    // the target distance bewtween hand to holding location, 
    float target_angle = 5f;       // the target angle distance between hand to up direction

    // 0.0 means not add this constraint, 1.0 means add it
    [HideInInspector]
    public float use_hand_angle = 0.0f; // use to check if need to add angle constraint(reward) into training
    [HideInInspector]
    public float use_comfort_pose = 0.0f; // use to check if need to add the comfort posture constraint into reward for training.
    [HideInInspector]
    public float use_hand_distance = 0.0f; // use to check  if need to add hand distance reward to the training
    [HideInInspector]
    public float use_palm_touch_board = 0.0f; // use to check if need to add palm touch board reward to the traning
    [HideInInspector]
    public float use_ball_balance = 0.0f; // use to check if need to add ball balance reward to the training
    [HideInInspector]
    public float use_board = 0.0f; // use to check if need to add board height reach reward to the training
    [HideInInspector]
    public float use_trimatch = 0.0f; // use to check if decide to use trimatch reward to replace: hand angle, hand distance, palm touch board reward functions.
    [HideInInspector]
    public float cam_rot_speed = 0.0f;
    [HideInInspector]
    public float cam_look_distance = 1.5f;

    Vector3 upper_arm_right_torque, lower_arm_right_torque, hand_right_torque, upper_arm_left_torque, lower_arm_left_torque, hand_left_torque;

    float hand_distance_ratio = 1f;
    float palm_touch_board_ratio = 1f;
    float hand_angle_ratio = 1f;
    float ball_balance_ratio = 1f;
    float board_ratio = 1f;
    float comfort_pose_ratio = 1f;

    float r_reach = 0.02f;
    float r_approch = 0.01f; //check scene5

    string log = ""; // use to hold the log information for check in exe window

    //private Dictionary<string, Vector3> jointsneutral = new Dictionary<string, Vector3>();
    Vector3 jn_upper_arm_right, jn_lower_arm_right, jn_hand_right, jn_upper_arm_left, jn_lower_arm_left, jn_hand_left;

    public override void Initialize()
    {
        woodyarea = GameObject.Find("WoodyArea").GetComponent<WoodyArea>();
        woodyarea.AreaInit();

        board = woodyarea.board;
        target = woodyarea.target;
        ball = woodyarea.ball;
        boardrigid = board.gameObject.GetComponent<Rigidbody>();
        ballrigid = ball.gameObject.GetComponent<Rigidbody>();

        leftholdjoint = lefthold.gameObject.GetComponent<CharacterJoint>();
        rightholdjoint = righthold.gameObject.GetComponent<CharacterJoint>();
        board_target_y = board.position.y + 0.1f;

        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;


        // each agent script will only need to focus on one agent's partial obersvation and action
        GetNeutralPos(this.name); // to get the neutral position of both arms for comfortable pose 
        //// init agent's instance, its location and rotation, and all jionts rigidbody
        woodyarea.AgentInitPoseGet(this.name);

        // get the reference of all needed joints
        GetJointsRef();
        //agentfacedir = agent.transform.forward;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();

        InitRewardCalculator();

        al1 = FindChildObjectwithName(this.transform, "l1");
        al2 = FindChildObjectwithName(this.transform, "l2");
        al3 = FindChildObjectwithName(this.transform, "l3");

        ar1 = FindChildObjectwithName(this.transform, "r1");
        ar2 = FindChildObjectwithName(this.transform, "r2");
        ar3 = FindChildObjectwithName(this.transform, "r3");

        // previous distance
        pre_l1 = Vector3.Distance(l1.position, al1.position);
        pre_l2 = Vector3.Distance(l2.position, al2.position);
        pre_l3 = Vector3.Distance(l3.position, al3.position);
        pre_r1 = Vector3.Distance(r1.position, ar1.position);
        pre_r2 = Vector3.Distance(r2.position, ar2.position);
        pre_r3 = Vector3.Distance(r3.position, ar3.position);
    }

    public override void CollectObservations(VectorSensor sensor) // 6+24+2+13*6+7*2=124
    {
        //6
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));
        // 7*3+3=24 //30
        sensor.AddObservation(board.position/2.0f);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(ball.position/2.0f);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(target.position/2.0f);
        sensor.AddObservation(target.rotation);
        sensor.AddObservation(ballrigid.velocity);
        //2
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        sensor.AddObservation(Mathf.Abs(board.position.y - board_target_y)/2.0f);
        //7+6=13 //45
        sensor.AddObservation(upper_arm_left.transform.position/2.0f);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity/20.0f);
        sensor.AddObservation(upper_arm_left.velocity/2.0f);
        //13 //58
        sensor.AddObservation(lower_arm_left.transform.position/2.0f);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity/20.0f); //68
        sensor.AddObservation(lower_arm_left.velocity/2.0f);//71
        //13 //71
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity/20.0f);
        sensor.AddObservation(hand_left.velocity/2.0f);
        //7 //78
        sensor.AddObservation(hand_left_end.position);
        sensor.AddObservation(hand_left_end.rotation);
        //13 //91
        sensor.AddObservation(upper_arm_right.transform.position/2.0f);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity/20.0f); //101
        sensor.AddObservation(upper_arm_right.velocity/2.0f); //104
        //13 //104
        sensor.AddObservation(lower_arm_right.transform.position/2.0f); //107
        sensor.AddObservation(lower_arm_right.transform.rotation); //111
        sensor.AddObservation(lower_arm_right.angularVelocity/20.0f); //114
        sensor.AddObservation(lower_arm_right.velocity/2.0f); //117
        //13 //117
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity/20.0f);
        sensor.AddObservation(hand_right.velocity/2.0f);
        //7 //124
        sensor.AddObservation(hand_right_end.position);
        sensor.AddObservation(hand_right_end.rotation);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var vectoraction = actionBuffers.ContinuousActions;
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude;
            //debug.log(i.tostring() + "th action= " + actions[i].tostring());
        }

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1] / 100, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4] / 100, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7] / 100, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10] / 100, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13] / 100, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16] / 100, vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

        log = "";
        countreach = 0;
        countdist = 0;

        ////add reward for lower energy of the arms
        //if (use_comfort_pose == 1.0)
        //{
        //    GetRewardComfortPose();
        //}

        //// add reward when the hands reach to the target position
        //if (use_hand_distance == 1.0)
        //{
        //    GetRewardHandDistance();
        //}

        //// add reward when the hands reach to the target angle
        //if (use_hand_angle == 1.0)
        //{
        //    GetRewardHandAngle();
        //}

        //if (use_palm_touch_board == 1.0)
        //{
        //    GetRewardPalmTouchBoard();
        //}

        if (use_board == 1.0)
        {
            GetRewardBoard();
        }

        if (use_ball_balance == 1.0)
        {
            GetRewardBallBalance();
        }

        AddReward(-1 / terminate_steps);

        if (use_trimatch == 1.0)
        {
            GetRewardTriMatch();
        }
        //Debug.DrawRay(lefthold.position, -lefthold.up * 0.5f, Color.red, Time.deltaTime, false);
        //Debug.DrawRay(righthold.position, -lefthold.up * 0.5f, Color.red, Time.deltaTime, false);

        //Debug.DrawRay(hand_left_end.position, -hand_left_end.up * 0.5f, Color.green, Time.deltaTime, false);
        //Debug.DrawRay(hand_right_end.position, -hand_right_end.up * 0.5f, Color.green, Time.deltaTime, false);
        //float angle = Vector3.Angle(-hand_left_end.up, -lefthold.up);
        //Debug.Log(this.name + " angle difference: " + angle);

        //Debug.DrawLine(l1.position, al1.position, Color.red);
        //Debug.DrawLine(l2.position, al2.position, Color.green);
        //Debug.DrawLine(l3.position, al3.position, Color.yellow);

        //Debug.DrawLine(r1.position, ar1.position, Color.red);
        //Debug.DrawLine(r2.position, ar2.position, Color.green);
        //Debug.DrawLine(r3.position, ar3.position, Color.yellow);
    }

    public void GetRewardHandDistance()
    {

        // make the reward to get closer to the 4 corners of the hand
        cur_leftdist = Vector3.Distance(lefthold.position, hand_left.transform.position);
        cur_rightdist = Vector3.Distance(righthold.position, hand_right.transform.position);

        float reward = 0.0f;
        if (cur_leftdist <= target_distance)
        {
            reward += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }
        if (cur_rightdist <= target_distance)
        {
            reward += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }

        //if (cur_leftdist < target_distance)
        //{
        //    reward += r_reach * hand_distance_ratio;
        //    AddReward(r_reach* hand_distance_ratio);
        //    countreach += 1;
        //    countdist += 1;

        //    log += "leftdist reached, ";
        //}
        //else
        //{
        //    if (cur_leftdist > pre_leftdist)
        //    {
        //        reward += -r_approch * hand_distance_ratio;
        //        AddReward(-r_approch * hand_distance_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * hand_distance_ratio;
        //        AddReward(r_approch * hand_distance_ratio);
        //    }


        //    log += "leftdist= " + cur_leftdist.ToString("F3") + ", ";
        //}

        //if (cur_rightdist < target_distance)
        //{
        //    reward += r_reach * hand_distance_ratio;
        //    AddReward(r_reach* hand_distance_ratio);
        //    countreach += 1;
        //    countdist += 1;

        //    log += "rightdist reached, ";
        //}
        //else
        //{
        //    if (cur_rightdist > pre_rightdist)
        //    {
        //        reward += -r_approch * hand_distance_ratio;
        //        AddReward(-r_approch * hand_distance_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * hand_distance_ratio;
        //        AddReward(r_approch * hand_distance_ratio);
        //    }

        //    log += "rightdist= " + cur_rightdist.ToString("F3") + ", ";

        //}

        //pre_leftdist = cur_leftdist;
        //pre_rightdist = cur_rightdist;

        //Debug.Log(this.name + " hand distance: " + reward.ToString());
        //Debug.Log(this.name + " left hand distance value: " + cur_leftdist);

        //Debug.DrawLine(lefthold.position, hand_left.transform.position, Color.blue);

    }

    public void GetRewardTriMatch()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var rew1 = 0f;

        if (cur_l1 + cur_l2 + cur_l3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }

        if (cur_r1 + cur_r2 + cur_r3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }

        var rew2 = 0f;

        if (cur_l1 <= pre_l1) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }

        if (cur_l2 <= pre_l2) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }

        if (cur_l3 <= pre_l3) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }

        if (cur_r1 <= pre_r1) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }

        if (cur_r2 <= pre_r2) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }

        if (cur_r3 <= pre_r3) { rew2 += r_approch; AddReward(r_approch); }
        else { rew2 += -r_approch; AddReward(-r_approch); }


        pre_l1 = cur_l1;
        pre_l2 = cur_l2;
        pre_l3 = cur_l3;
        pre_r1 = cur_r1;
        pre_r2 = cur_r2;
        pre_r3 = cur_r3;

        Debug.Log(this.name + " tri distance reward: " + "rew1= " + rew1 + " rew2= " + rew2);
    }

    public void GetRewardTriMatch1()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var rew1 = 0f;
        if (cur_l1 + cur_l2 + cur_l3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }

        if (cur_r1 + cur_r2 + cur_r3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * hand_distance_ratio;
            AddReward(r_reach * hand_distance_ratio);
        }

        AddReward((0.01f - cur_l1));
        AddReward((0.01f - cur_l2));
        AddReward((0.01f - cur_l3));

        AddReward((0.01f - cur_r1));
        AddReward((0.01f - cur_r2));
        AddReward((0.01f - cur_r3));

        //var limit = 0.01f;
        //if (cur_l1 < limit) cur_l1 = limit;
        //if (cur_l2 < limit) cur_l2 = limit;
        //if (cur_l3 < limit) cur_l3 = limit;
        //if (cur_r1 < limit) cur_r1 = limit;
        //if (cur_r2 < limit) cur_r2 = limit;
        //if (cur_r3 < limit) cur_r3 = limit;

        var rew2 = 0f;
        //var upper_limit = 0.005f;
        //AddReward(upper_limit / cur_l1);
        //AddReward(upper_limit / cur_l2);
        //AddReward(upper_limit / cur_l3);

        //AddReward(upper_limit / cur_r1);
        //AddReward(upper_limit / cur_r2);
        //AddReward(upper_limit / cur_r3);

        //rew2 += upper_limit / cur_l1 + upper_limit / cur_l2 + upper_limit / cur_l3 + upper_limit / cur_r1 + upper_limit / cur_r2 + upper_limit / cur_r3;
        rew2 = 0.06f - cur_l1 - cur_l2 - cur_l3 - cur_r1 - cur_r2 - cur_r3;
        Debug.Log(this.name + " tri distance: " + cur_l1 + "," + cur_l2 + "," + cur_l3 + "," + cur_r1 + "," + cur_r2 + "," + cur_r3);
        Debug.Log("tri distance reward: " + "rew1= " + rew1 + " rew2= " + rew2);
    }

    public void GetRewardHandAngle()

    {
        //cur_leftang = Vector3.Angle( -limbends["hand_left_end"].right, -lefthold.up);
        //cur_rightang = Vector3.Angle(-limbends["hand_right_end"].right, -righthold.up);

        cur_leftang = Vector3.Angle(-hand_left_end.up, -lefthold.up);
        cur_rightang = Vector3.Angle(-hand_right_end.up, -righthold.up);

        float reward = 0.0f;
        if (cur_leftang <= target_angle)
        {
            reward += r_reach * hand_angle_ratio;
            AddReward(r_reach * hand_angle_ratio);
        }

        if (cur_rightang <= target_angle)
        {
            reward += r_reach * hand_angle_ratio;
            AddReward(r_reach * hand_angle_ratio);
        }


        ////if (-hand_left_end.localEulerAngles.y > 180)
        ////{
        ////    cur_leftrotup = Mathf.Abs(-hand_left_end.localEulerAngles.y - 360);
        ////}
        ////else
        ////{
        ////    cur_leftrotup = Mathf.Abs(-hand_left_end.localEulerAngles.y - 0);
        ////}

        ////if (-hand_right_end.localEulerAngles.y > 180)
        ////{
        ////    cur_rightrotup = Mathf.Abs(-hand_right_end.localEulerAngles.y - 360);
        ////}
        ////else
        ////{
        ////    cur_rightrotup = Mathf.Abs(-hand_right_end.localEulerAngles.y - 0);
        ////}

        ////if (cur_leftrotup < target_angle)
        ////{
        ////    reward += r_reach * hand_angle_ratio;
        ////    AddReward(r_reach * hand_angle_ratio);
        ////}

        ////if (cur_rightrotup < target_angle)
        ////{
        ////    reward += r_reach * hand_angle_ratio;
        ////    AddReward(r_reach * hand_angle_ratio);
        ////}

        //if (cur_leftang < target_angle)
        //{
        //    reward += r_reach * hand_angle_ratio;
        //    AddReward(r_reach * hand_angle_ratio);
        //    countreach += 1;

        //    log += "leftangle reached, ";
        //}
        //else
        //{
        //    if (cur_leftang > pre_leftang)
        //    {
        //        reward += -r_approch * hand_angle_ratio;
        //        AddReward(-r_approch * hand_angle_ratio);
        //    }

        //    else
        //    {
        //        reward += r_approch * hand_angle_ratio;
        //        AddReward(r_approch * hand_angle_ratio);
        //    }


        //    log += "leftangle= " + cur_leftang.ToString("F1") + ", ";

        //}

        //if (cur_rightang < target_angle)
        //{
        //    reward += r_reach * hand_angle_ratio;
        //    AddReward(r_reach * hand_angle_ratio);
        //    countreach += 1;

        //    log += "rightangle reached, ";
        //}
        //else
        //{
        //    if (cur_rightang > pre_rightang)
        //    {
        //        reward += -r_approch * hand_angle_ratio;
        //        AddReward(-r_approch * hand_angle_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * hand_angle_ratio;
        //        AddReward(r_approch * hand_angle_ratio);
        //    }


        //    log += "rightangle= " + cur_rightang.ToString("F1") + ", ";

        //}

        //pre_leftang = cur_leftang;
        //pre_rightang = cur_rightang;

        //Debug.Log(this.name + " hand angle: " + reward.ToString());
    }

    public void GetRewardPalmTouchBoard()
    {
        float reward = 0.0f;
        if (Physics.Raycast(hand_left_end.position, hand_left_end.TransformDirection(-Vector3.up), out lefthit, Mathf.Infinity) && lefthit.collider.tag == "tray")
        {
            reward += r_reach * palm_touch_board_ratio;
            AddReward(r_reach * palm_touch_board_ratio);
            countreach += 1;

            log += "leftpalm reached, ";
        }
        //else
        //{
        //    reward += -r_reach * palm_touch_board_ratio;
        //    AddReward(-r_reach * palm_touch_board_ratio);
        //    log += "leftpalm not reach, ";
        //}

        if (Physics.Raycast(hand_right_end.position, hand_right_end.TransformDirection(-Vector3.up), out righthit, Mathf.Infinity) && righthit.collider.tag == "tray")
        {
            reward += r_reach * palm_touch_board_ratio;
            AddReward(r_reach * palm_touch_board_ratio);
            countreach += 1;

            log += "rightpalm reached, ";
        }
        //else
        //{
        //    reward += -r_reach * palm_touch_board_ratio;
        //    AddReward(-r_reach * palm_touch_board_ratio);
        //    log += "rightpalm not reach, ";
        //}

        //Debug.Log(this.name + " palm touch board: " + reward.ToString());
    }

    public bool ConnectBoard()
    {
        var checkconnect = false;
        if (use_hand_angle == 1)
        {
            if (countdist == 2)//(countreach == 6)
            {
                leftholdjoint.connectedBody = hand_left;
                rightholdjoint.connectedBody = hand_right;
                checkconnect = true;
            }
        }

        return checkconnect;
    }

    public void GetRewardBallBalance()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        float reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio);
            countreach += 1;

            log += "ball reached, ";
        }

        //AddReward((0.3f - cur_balldist) * 0.1f);

        if (cur_balldist < 0.05f) cur_balldist = 0.05f;
        AddReward(0.025f / cur_balldist);

        Debug.Log(this.name + " ball distance reward: " + " rew1= " + reward + " rew2= " + 0.025f / cur_balldist);
        //else
        //{
        //    if (cur_balldist > pre_balldist)
        //    {
        //        reward += -r_approch * ball_balance_ratio;
        //        AddReward(-r_approch * ball_balance_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * ball_balance_ratio;
        //        AddReward(r_approch * ball_balance_ratio);
        //    }

        //    log += "balldist= " + cur_balldist.ToString("F3") + ", ";
        //}
        //pre_balldist = cur_balldist;

        //Debug.Log(this.name + " ball balance: " + reward.ToString());
    }

    public void GetRewardBoard()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        //cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;

        //if (cur_boardang < target_angle && cur_boarddist_y < target_distance)
        //{
        //    reward += r_reach * board_ratio;
        //    AddReward(r_reach * board_ratio);
        //}

        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }

        if (cur_boarddist_y < target_distance) cur_boarddist_y = target_distance;
        AddReward(0.01f / cur_boarddist_y);

        //if (cur_boardang < target_angle)
        //{
        //    reward += r_reach * board_ratio;
        //    AddReward(r_reach * board_ratio);
        //    countreach += 1;

        //    log += "boardangle reached, ";
        //}
        //else
        //{
        //    if (cur_boardang > pre_boardang)
        //    {
        //        reward += -r_approch * board_ratio;
        //        AddReward(-r_approch * board_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * board_ratio;
        //        AddReward(r_approch * board_ratio);
        //    }

        //    log += "boardangle= " + cur_boardang.ToString("F1") + ", ";
        //}
        //pre_boardang = cur_boardang;

        //if (cur_boarddist_y < target_distance)
        //{
        //    reward += r_reach * board_ratio;
        //    AddReward(r_reach * board_ratio);
        //    countreach += 1;

        //    log += "boarddist reached, ";
        //}
        //else
        //{
        //    if (cur_boarddist_y > pre_boarddist_y)
        //    {
        //        reward += -r_approch * board_ratio;
        //        AddReward(-r_approch * board_ratio);
        //    }
        //    else
        //    {
        //        reward += r_approch * board_ratio;
        //        AddReward(r_approch * board_ratio);
        //    }

        //    log += "boarddist= " + cur_boarddist_y.ToString("F3") + ", ";
        //}

        //pre_boarddist_y = cur_boarddist_y;

        Debug.Log(" board dist reward: " + "rew1: " +  reward.ToString() + "rew2: " + 0.01f / cur_boarddist_y);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

        float torqueforce = 10f;
        var actions = actionsOut.ContinuousActions;

        if (Input.GetKey(KeyCode.Q)) actions[0] = torqueforce;
        if (Input.GetKey(KeyCode.A)) actions[0] = -torqueforce;
        if (Input.GetKey(KeyCode.W)) actions[1] = torqueforce;
        if (Input.GetKey(KeyCode.S)) actions[1] = -torqueforce;
        if (Input.GetKey(KeyCode.E)) actions[2] = torqueforce;
        if (Input.GetKey(KeyCode.D)) actions[2] = -torqueforce;
        if (Input.GetKey(KeyCode.R)) actions[3] = torqueforce;
        if (Input.GetKey(KeyCode.F)) actions[3] = -torqueforce;
        if (Input.GetKey(KeyCode.T)) actions[4] = torqueforce;
        if (Input.GetKey(KeyCode.G)) actions[4] = -torqueforce;
        if (Input.GetKey(KeyCode.Y)) actions[5] = torqueforce;
        if (Input.GetKey(KeyCode.H)) actions[5] = -torqueforce;
        if (Input.GetKey(KeyCode.U)) actions[6] = torqueforce;
        if (Input.GetKey(KeyCode.J)) actions[6] = -torqueforce;
        if (Input.GetKey(KeyCode.I)) actions[7] = torqueforce;
        if (Input.GetKey(KeyCode.K)) actions[7] = -torqueforce;
        if (Input.GetKey(KeyCode.O)) actions[8] = torqueforce;
        if (Input.GetKey(KeyCode.L)) actions[8] = -torqueforce;
        if (Input.GetKey(KeyCode.Z)) actions[9] = torqueforce;
        if (Input.GetKey(KeyCode.X)) actions[9] = -torqueforce;
        if (Input.GetKey(KeyCode.C)) actions[10] = torqueforce;
        if (Input.GetKey(KeyCode.V)) actions[10] = -torqueforce;
        if (Input.GetKey(KeyCode.B)) actions[11] = torqueforce;
        if (Input.GetKey(KeyCode.N)) actions[11] = -torqueforce;
        if (Input.GetKey(KeyCode.M)) actions[12] = torqueforce;
        if (Input.GetKey(KeyCode.P)) actions[12] = -torqueforce;
        if (Input.GetKey(KeyCode.LeftArrow)) actions[13] = torqueforce;
        if (Input.GetKey(KeyCode.RightArrow)) actions[13] = -torqueforce;

    }

    public override void OnEpisodeBegin()
    {

        woodyarea.AreaReset();
        woodyarea.AgentInitPoseReset(this.name);
        //reset the force ratio here as a accessible paramters in python api
        SetResetParameters();
        InitRewardCalculator();

        // break the holder joint and reset the board info
        leftholdjoint.connectedBody = null;
        rightholdjoint.connectedBody = null;

        //Debug.unityLogger.logEnabled = false;
        // this part output on console has showed that python lower level code could change those parameters 
        //Debug.Log("terminate_step: " + terminate_steps + " use_hand_angle: " + use_hand_angle + " use_comfort_pose: " + use_comfort_pose);
        //Debug.Log("use_hand_distance: " + use_hand_distance + " use_palm_touch_board: " + use_palm_touch_board + " use_ball_balance: " + use_ball_balance);
        //Debug.Log("use_board: " + use_board + " comfort_pose_ratio: " + comfort_pose_ratio + " board_ratio: " + board_ratio);
        if (use_trimatch == 1.0f)
        {
            use_hand_distance = 0.0f;
            use_hand_angle = 0.0f;
            use_palm_touch_board = 0.0f;
        }
    }

    void SetResetParameters()
    {
        // general parameters
        force_magnitude = m_ResetParams.GetWithDefault("force_magnitude", 500.0f);
        terminate_steps = m_ResetParams.GetWithDefault("terminate_steps", 500f);

        // target distance and anlge value, could also be used for curriculum learning purpose
        target_distance = m_ResetParams.GetWithDefault("target_distance", 0.02f);
        target_angle = m_ResetParams.GetWithDefault("target_angle", 5f);

        // control parameters for which reward to use for training
        use_hand_angle = m_ResetParams.GetWithDefault("use_hand_angle", 0.0f);
        use_comfort_pose = m_ResetParams.GetWithDefault("use_comfort_pose", 0.0f);
        use_hand_distance = m_ResetParams.GetWithDefault("use_hand_distance", 1.0f);
        use_palm_touch_board = m_ResetParams.GetWithDefault("use_palm_touch_board", 0.0f);
        use_ball_balance = m_ResetParams.GetWithDefault("use_ball_balance", 0.0f);
        use_board = m_ResetParams.GetWithDefault("use_board", 0.0f);
        use_trimatch = m_ResetParams.GetWithDefault("use_trimatch", 0.0f);

        // reward ratio parameters for controlling the weight ratio of each reward
        hand_distance_ratio = m_ResetParams.GetWithDefault("hand_distance_ratio", 3f);
        hand_angle_ratio = m_ResetParams.GetWithDefault("hand_angle_ratio", 2f);
        palm_touch_board_ratio = m_ResetParams.GetWithDefault("palm_touch_board_ratio", 2f);
        ball_balance_ratio = m_ResetParams.GetWithDefault("ball_balance_ratio", 2f);
        board_ratio = m_ResetParams.GetWithDefault("board_ratio", 2f);
        comfort_pose_ratio = m_ResetParams.GetWithDefault("comfort_pose_ratio", 1f);

        // for camera
        cam_rot_speed = m_ResetParams.GetWithDefault("cam_rot_speed", 0f);
        cam_look_distance = m_ResetParams.GetWithDefault("cam_look_distance", 1.5f);
    }

    void InitRewardCalculator()
    {
        pre_leftdist = Vector3.Distance(lefthold.position, hand_left.transform.position);
        pre_rightdist = Vector3.Distance(righthold.position, hand_right.transform.position);

        pre_leftang = Vector3.Angle(-hand_left_end.up, Vector3.up);
        pre_rightang = Vector3.Angle(-hand_right_end.up, Vector3.up);

        pre_balldist = Vector3.Distance(target.position, ball.position);

        pre_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        pre_boardang = Vector3.Angle(board.up, Vector3.up);
    }

    void GetNeutralPos(string agentname)
    {
        if (this.name == "Agent1")
        {
            jn_upper_arm_right = new Vector3(-0.1f, 1.4f, 0.5f);
            jn_lower_arm_right = new Vector3(-0.1f, 1.1f, 0.5f);
            jn_hand_right = new Vector3(-0.1f, 0.8f, 0.5f);

            jn_upper_arm_left = new Vector3(0.1f, 1.4f, 0.5f);
            jn_lower_arm_left = new Vector3(0.1f, 1.1f, 0.5f);
            jn_hand_left = new Vector3(0.1f, 0.8f, 0.5f);
        }

        if (this.name == "Agent2")
        {
            jn_upper_arm_right = new Vector3(0.1f, 1.4f, -0.5f);
            jn_lower_arm_right = new Vector3(0.1f, 1.1f, -0.5f);
            jn_hand_right = new Vector3(0.1f, 0.8f, -0.5f);

            jn_upper_arm_left = new Vector3(-0.1f, 1.4f, -0.5f);
            jn_lower_arm_left = new Vector3(-0.1f, 1.1f, -0.5f);
            jn_hand_left = new Vector3(-0.1f, 0.8f, -0.5f);
        }
    }

    public void GetRewardComfortPose()
    {
        var jointdisplacement = 0.0f;

        jointdisplacement += Vector3.Distance(upper_arm_right.transform.position, jn_upper_arm_right);
        jointdisplacement += Vector3.Distance(lower_arm_right.transform.position, jn_lower_arm_right);
        jointdisplacement += Vector3.Distance(hand_right.transform.position, jn_hand_right);
        jointdisplacement += Vector3.Distance(upper_arm_left.transform.position, jn_upper_arm_left);
        jointdisplacement += Vector3.Distance(lower_arm_left.transform.position, jn_lower_arm_left);
        jointdisplacement += Vector3.Distance(hand_left.transform.position, jn_hand_left);

        float reward = 0.0f;
        reward = jointdisplacement / 4f * 0.001f * comfort_pose_ratio;
        AddReward(jointdisplacement / 4f * 0.001f * comfort_pose_ratio);

        Debug.Log(this.name + " comfort pose reward: " + reward.ToString());

    }

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

}

