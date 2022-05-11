
using UnityEngine;
using Unity.MLAgents;
[System.Serializable]
public class TrainTest_menv : MonoBehaviour
{
    WoodyAgent_menv agent;
    GameObject[] woodyareas = new GameObject[16];
    EnvironmentParameters m_ResetParams;
    [HideInInspector]
    public float multi_env=1f;

    Transform agent1, agent2;
    int index;

    Transform cam, maincam;

    float lookdistance, speed;

    Vector3 cam_rel_pos;

    int lookatenv = 13;
    void Start()
    {
        cam= GameObject.Find("Camera").transform;
        maincam = GameObject.Find("Main Camera").transform;

        // Debug.Log("traintest start--------------");
        m_ResetParams = Academy.Instance.EnvironmentParameters;
        multi_env = m_ResetParams.GetWithDefault("multi_env", 0f);
        lookdistance = m_ResetParams.GetWithDefault("cam_look_distance", 2f);
        speed = m_ResetParams.GetWithDefault("cam_rot_speed", 0.03f);

        foreach (GameObject gobj in GameObject.FindGameObjectsWithTag("woodyarea"))
        {
            string name = gobj.name.Split(')')[0];
            string num = name.Substring(11);
            int.TryParse(num, out index);
            woodyareas[index] = gobj;
        }
        
        if(multi_env==1f)
        {
            for(int i = 0; i < 16; i++)
            { 
                woodyareas[i].SetActive(true);
                if(!woodyareas[i].GetComponent<WoodyArea_menv>())
                    woodyareas[i].AddComponent<WoodyArea_menv>();
                var warea = woodyareas[i].GetComponent<WoodyArea_menv>();
                warea.multi_env = multi_env;
                agent1 = warea.agent1;
                agent2 = warea.agent2;
                
                if(!agent2.GetComponent<WoodyAgent_menv>())
                {
                    agent2.gameObject.AddComponent<WoodyAgent_menv>();
                    agent2.gameObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                    agent2.GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod = 5;
                    agent2.GetComponent<Unity.MLAgents.DecisionRequester>().TakeActionsBetweenDecisions = true;
                }

                if(!agent1.GetComponent<WoodyAgent_menv>())
                {
                    agent1.gameObject.AddComponent<WoodyAgent_menv>();
                    agent1.gameObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                    agent1.GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod = 5;
                    agent1.GetComponent<Unity.MLAgents.DecisionRequester>().TakeActionsBetweenDecisions = true;   
                    
                }

                agent1.GetComponent<WoodyAgent_menv>().otheragent = agent2.GetComponent<WoodyAgent_menv>();
                agent2.GetComponent<WoodyAgent_menv>().otheragent = agent1.GetComponent<WoodyAgent_menv>();

                
            }
        }
        else
        {
            for(int i = 0; i < 16; i++)
            {
                if (i != lookatenv)
                {
                    // if(woodyareas[i].GetComponent<WoodyArea_menv>())
                    //     Destroy(woodyareas[i].GetComponent<WoodyArea_menv>());
                    woodyareas[i].SetActive(false);
                }
                else
                {
                    
                    woodyareas[i].SetActive(true);
                    if(!woodyareas[i].GetComponent<WoodyArea_menv>())
                        woodyareas[i].AddComponent<WoodyArea_menv>();
                    var warea = woodyareas[i].GetComponent<WoodyArea_menv>();
                    warea.multi_env = multi_env;

                    agent1 = warea.agent1;
                    agent2 = warea.agent2;
                    
                    if(!agent1.GetComponent<WoodyAgent_menv>())
                    {
                        agent1.gameObject.AddComponent<WoodyAgent_menv>();
                        agent1.gameObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                        agent1.GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod = 5;
                        agent1.GetComponent<Unity.MLAgents.DecisionRequester>().TakeActionsBetweenDecisions = true;   
                    }
                    
                    if(!agent2.GetComponent<WoodyAgent_menv>())
                    {
                        agent2.gameObject.AddComponent<WoodyAgent_menv>();
                        agent2.gameObject.AddComponent<Unity.MLAgents.DecisionRequester>();
                        agent2.GetComponent<Unity.MLAgents.DecisionRequester>().DecisionPeriod = 5;
                        agent2.GetComponent<Unity.MLAgents.DecisionRequester>().TakeActionsBetweenDecisions = true;
                    }

                    agent1.GetComponent<WoodyAgent_menv>().otheragent = agent2.GetComponent<WoodyAgent_menv>();
                    agent2.GetComponent<WoodyAgent_menv>().otheragent = agent1.GetComponent<WoodyAgent_menv>();
                }
            }
        }

        // Debug.Log("traintest start done--------------" + multi_env);
        
        if(multi_env == 1f)
        {
            cam.gameObject.SetActive(true);
            maincam.gameObject.SetActive(false);
        }
        else
        {
            cam.gameObject.SetActive(false);
            maincam.gameObject.SetActive(true);
            woodyareas[lookatenv].GetComponent<WoodyArea_menv>().ground.transform.localScale = new Vector3(4,0.1f,4);
            cam_rel_pos = new Vector3(woodyareas[lookatenv].transform.position.x, woodyareas[lookatenv].transform.position.y+0.8f, woodyareas[lookatenv].transform.position.z);
            maincam.transform.GetComponent<RotateCam_menv>().lookdistance = lookdistance;
            maincam.transform.GetComponent<RotateCam_menv>().speed = speed;
            maincam.transform.GetComponent<RotateCam_menv>().cam_rel_pos = cam_rel_pos;
        }
    }
}



