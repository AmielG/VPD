﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectTimer
{
    public GameObject obj;
    public float timer = 0.0f;
}

public class RaceCheckPoint : MonoBehaviour
{
    public List<ObjectTimer> required_col = new List<ObjectTimer>();

    int m_iCheckPoint = 0;

    public void SetCheckpointIndex(int i)
    {
        m_iCheckPoint = i;
    }

    public void Reset()
    {
        required_col.Clear();
    }

    public void AddRequiredHit(GameObject ob, float required_col_time)
    {
        ObjectTimer ot = new ObjectTimer();
        ot.obj = ob;
        ot.timer = required_col_time;
        required_col.Add(ot);
    } 

    public bool RemoveBody(GameObject go)
    {
        for (int iT = 0; iT < required_col.Count; iT++)
        {
            if(required_col[iT].obj == go)
            {
                required_col.RemoveAt(iT);
                return true;
            }
        }

        return false;
    }

    void OnTriggerEnter(Collider col)
    {
        bool isBody = col.gameObject.name == "body";

        for (int iO = 0; iO < required_col.Count; iO++)
        {
            if(required_col[iO].obj == col.gameObject)
            {
                RaceManager rm = GameObject.FindObjectOfType<RaceManager>();
                rm.OnHitCheckPoint(col.gameObject, m_iCheckPoint);
                break;
            }
        }     
    }

    // Update is called once per frame
    void Update()
    {
        foreach (ObjectTimer ot in required_col)
            if (ot.timer > 0.0f)
            {
                ot.timer -= Time.deltaTime;

                if(ot.timer <= 0.0f)
                {
                    ot.timer = 0.0f;
                    RaceManager rm = GameObject.FindObjectOfType<RaceManager>();

                    if (ot.obj != null)
                    {
                        rm.OnCheckPointTimedOut(ot.obj, m_iCheckPoint);
                        break;
                    }
                }
            }
    }
}
