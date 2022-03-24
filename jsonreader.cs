using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.IO;
using Newtonsoft.Json;

public class jsonreader : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        string fileName = @"C:\Users\Max\Desktop\DTU_Softwareteknologi\6.Semester2021\Bachelorprojekt\bachelor_project_teledyne\7k_data_extracted.json";
        string jsonString = File.ReadAllText(fileName);
        Sonar sonarData = JsonConvert.DeserializeObject<Sonar>(jsonString);

        print($"no_pings : {sonarData.no_pings}");
        print($"pings: {sonarData.pings}");
        List<Vector3[]> pings = new List<Vector3[]>();
        for (int i = 0; i < 1; i++)
        {
            Vector3[] points = new Vector3[sonarData.pings[i].no_points];
            for (int j = 0; j < sonarData.pings[i].no_points; j++)
            {
                //print($"{sonarData.pings[i].coords_y[j]}");
                Vector3 coord = new Vector3((float)sonarData.pings[i].coords_x[j], (float)sonarData.pings[i].coords_y[j], (float)sonarData.pings[i].coords_z[j]);
                points[j] = coord;
            }
            pings.Add(points);
        }
        print("hej");
        print(pings[0][0].ToString());
        

    }

    // Update is called once per frame
    void Update()
    {
        
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

