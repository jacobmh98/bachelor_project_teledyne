using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Newtonsoft.Json;
using System.IO;

public class ObjData
{
    public Vector3 pos;
    public Vector3 scale;
    public Quaternion rot;

    public Matrix4x4 matrix
    {
        get
        {
            return Matrix4x4.TRS(pos, rot, scale);
        }
    }

    public ObjData(Vector3 pos, Vector3 scale, Quaternion rot)
    {
        this.pos = pos;
        this.scale = scale;
        this.rot = rot;
    }
}

public class Spawner : MonoBehaviour
{
    public int instances;
    public Vector3 maxPos;
    public Mesh objMesh;
    public Material objMat;
    public Vector3[] positions;

    private List<List<ObjData>> batches = new List<List<ObjData>>();

    void Start()
    {
        string fileName = @"C:\Users\jacob\OneDrive - Danmarks Tekniske Universitet\6. semester\Bachelor Project\7k_data_extracted.json";
        string jsonString = File.ReadAllText(fileName);
        Sonar sonarData = JsonConvert.DeserializeObject<Sonar>(jsonString);

        print($"no_pings : {sonarData.no_pings}");
        print($"pings: {sonarData.pings}");
        List<Vector3[]> pings = new List<Vector3[]>();
        int no_pings = 0;

        int no_points_total = 0;
        for (int i = 0; i < 150; i++)
        {
            //if (i % 10 != 0)
              //  continue;

            Vector3[] points = new Vector3[sonarData.pings[i].no_points];
            no_points_total += sonarData.pings[i].no_points;
            for (int j = 0; j < sonarData.pings[i].no_points; j++)
            {
                //print($"{sonarData.pings[i].coords_y[j]}");
                Vector3 coord = new Vector3((float)sonarData.pings[i].coords_x[j] * 100, (float)sonarData.pings[i].coords_z[j], (float)sonarData.pings[i].coords_y[j]);
                points[j] = coord;
            }
            no_points_total += points.Length;
            pings.Add(points);
            no_pings++;
        }

        instances = no_points_total;

        positions = new Vector3[no_points_total];

        int point_i = 0;

        for (int i = 0; i < no_pings; i++)
        {
            
            for (int j = 0; j < pings[i].Length; j++)
            {
                positions[point_i] = pings[i][j];
                point_i++;
            }
        }

        int batchIndexNum = 0;
        List<ObjData> currBatch = new List<ObjData>();
        for(int i = 0; i < instances; i++)
        {
            AddObj(currBatch, i);
            batchIndexNum++;
            if(batchIndexNum >= 1000)
            {
                batches.Add(currBatch);
                currBatch = BuildNewBatch();
                batchIndexNum = 0;
            }
        }
    }

    void Update()
    {
        RenderBatches();
    }

    private void AddObj(List <ObjData> currBatch, int i)
    {
        Vector3 position = (positions[i]);
        currBatch.Add(new ObjData(position, new Vector3(1f,1f,1f), Quaternion.identity));
    }

    private List<ObjData> BuildNewBatch()
    {
        return new List<ObjData>();
    }

    private void RenderBatches()
    {
        foreach(var batch in batches)
        {
            Graphics.DrawMeshInstanced(objMesh, 0, objMat, batch.Select((a) => a.matrix).ToList());
        }
    }

    public class Ping
    {
        public int pingID { get; set; }
        public int no_points { get; set; }
        public List<double> ping_coord { get; set; }
        public List<double> coords_x { get; set; }
        public List<double> coords_y { get; set; }
        public List<double> coords_z { get; set; }
    }

    public class Sonar
    {
        public int no_pings { get; set; }
        public List<Ping> pings { get; set; }
    }
}
