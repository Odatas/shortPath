using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class PathCalculator : MonoBehaviour
{
    //<summary>
    //Calculates the closest worker for a task by limiting the path calculations of navmeshagents to the workers with the smallest vector3 distance. 
    //
    //</summary>
    public void findTaskForWorker(List<NavMeshAgent> idleWorker,Vector3 targetPosition, int maxPath, float threshhold)
    {
        //maxPath=0 means no limit on how many workers we calculate. 
        if (maxPath == 0)
            maxPath = idleWorker.Count;
        //get Task to get the postion of the task
        

        List<float> distancelist = new List<float>();
        float distance = Vector3.Distance(idleWorker[0].transform.position, targetPosition);
        NavMeshPath path = new NavMeshPath();

        //calculate Vector3 distances from targetposition to all workers
        //Should not take long
        for (int i = 0; i < idleWorker.Count; i++)
        {
            distancelist.Add(Vector3.Distance(idleWorker[i].transform.position, targetPosition));
        }

        float min = 0;
        float newmin = 0;
        int index = 0;
        float pathlength = 0;
        int minWorkerIndex = 0;
        float minPath = 0;
        //maxtimer is the ammount in seconds the function waits for a pathcompletion. 
        //If it takes longer that path will be skipped.
        float maxtimer = 0.5f;


        if (maxPath > idleWorker.Count)
            maxPath = idleWorker.Count;

        //calculate maxPath ammount of paths to compare
        for (int i = 0; i < maxPath; i++)
        {
            //search for lowest vector3.distance
            for (int j = 0; j < distancelist.Count; j++)
            {
                //distance must be bigger than previous found distance and smaller than disstance allready saved
                if (distancelist[j] < newmin && distancelist[j] > min || newmin == 0 && distancelist[j] > min)
                {
                    newmin = distancelist[j];
                    index = j;
                }
            } 

            //Optimization for a navmesh in the y=0 plane. Remove when you have other navmesh. 
            targetPosition.y = 0;
            GetPath(path, idleWorker[index].transform.position, targetPosition, NavMesh.AllAreas);
            //waits up to .5 seconds for a path to be completted. Usually is faster.This is a fallback to not lock the mainthread when no path can be found. 
            while (path.status == NavMeshPathStatus.PathInvalid && maxtimer > 0)
            {
                maxtimer -= Time.deltaTime;
            }
            maxtimer = 0.5f;
            pathlength = GetPathLength(path);
            //When a path is found that meets the threshold we stop.
            //For example threshhold=2 means path double the distance as vector3.distance is ok.
            if (pathlength / distancelist[index] < threshhold)
            {
                idleWorker[index].SetDestination(targetPosition);
                return;
            }

            min = newmin;
            newmin = 0;
            //TODO Test if pathlength can become 0
            //If we found a path that is shorter than the prvious shortes path we save index and pathlength.
            if (pathlength < minPath || minPath == 0)
            {
                minWorkerIndex = index;
                minPath = pathlength;
            }
        }
        //after calculation set the destination for the worker with the shortes path
        idleWorker[minWorkerIndex].SetDestination(targetPosition);
        return;
    }

    //<summary>
    //Calculates the navhmesh path from between two positions.
    //</summary>
    public static bool GetPath(NavMeshPath path, Vector3 fromPos, Vector3 toPos, int passableMask)
    {
        path.ClearCorners();

        if (NavMesh.CalculatePath(fromPos, toPos, passableMask, path) == false)
            return false;

        return true;
    }


    //<summary>
    //Calculates for a given path the 
    //</summary>

    public static float GetPathLength(NavMeshPath path)
    {
        float lng = 0.0f;

        if ((path.status != NavMeshPathStatus.PathInvalid))
        {
            for (int i = 1; i < path.corners.Length; ++i)
            {
                lng += Vector3.Distance(path.corners[i - 1], path.corners[i]);
            }
        }
        return lng;
    }


}
}
