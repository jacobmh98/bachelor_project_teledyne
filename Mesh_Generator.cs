using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static jsonreader;

[RequireComponent(typeof(MeshFilter))]
public class Mesh_Generator : MonoBehaviour
{
    Vector3[] vertices;
    int[] triangles;
    Mesh mesh;
    // Start is called before the first frame update
    void Start()
    {
        print("In MeshGenerator");
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        CreateShape();

    }

    void CreateShape()
    {
        JsonReader json = new JsonReader();
        List<Vector3[]> pings = json.getPings();
        List<Vector3> coords = new List<Vector3>();

        int xSize = pings.Count;
        int zSize = 510;


        for(int i = 0; i < xSize; i++)
        {
            for (int j = 0; j < zSize; j++)
            {
                coords.Add(pings[i][j]);
            }
        }
        print("bruh");
        print(coords[0]);
        print(coords.Count);
        vertices = new Vector3[(xSize + 1) * (zSize + 1)];
        for (int i = 0; i < coords.Count; i++)
        {
            vertices[i] = coords[i];
        }
        print(vertices[0]);

        int vert = 0;
        int tris = 0;
        triangles = new int[xSize* zSize * 6];

       
        for (int z = 0; z < zSize; z++)
        {
            for (int x = 0; x < xSize; x++)
            {
                triangles[tris + 0] = vert + 0;
                triangles[tris + 1] = vert + xSize + 1;
                triangles[tris + 2] = vert + 1;
                triangles[tris + 3] = vert + 1;
                triangles[tris + 4] = vert + xSize + 1;
                triangles[tris + 5] = vert + xSize + 2;
                vert++;
                tris += 6;
            }
            vert++;
        }
        print(triangles[triangles.Length-1]);
        
    }

    void UpdateMesh()
    {
        mesh.Clear();

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        
        mesh.RecalculateNormals();
    }
    /*
    private void OnDrawGizmos()
    {
        if(vertices == null)
        {
            return;
        }
        for(int i = 0; i < vertices.Length; i++)
        {
            Gizmos.DrawSphere(vertices[i], .1f);
        }

    }
    */
    // Update is called once per frame
    void Update()
    {
        UpdateMesh();
    }
}
