using UnityEngine;
using UnityEditor;
using System.IO;
using System.Collections.Generic;

public class AutoAssignTextures : EditorWindow
{
    [MenuItem("Tools/Auto-Assign Portal Textures")]
    static void Assign()
    {
        string matFolder = "Assets/PortalMesh/test-chamber-00-01-v2/source/materials";

        string[] matGuids = AssetDatabase.FindAssets("t:Material", new[] { matFolder });
        int fixed_count = 0;

        foreach (string guid in matGuids)
        {
            string matPath = AssetDatabase.GUIDToAssetPath(guid);
            Material mat = AssetDatabase.LoadAssetAtPath<Material>(matPath);
            if (mat == null) continue;

            bool changed = false;

            if (mat.HasProperty("_BaseColor"))
            {
                Color c = mat.GetColor("_BaseColor");
                if (c != Color.white)
                {
                    mat.SetColor("_BaseColor", Color.white);
                    changed = true;
                }
            }
            if (mat.HasProperty("_Color"))
            {
                Color c = mat.GetColor("_Color");
                if (c != Color.white)
                {
                    mat.SetColor("_Color", Color.white);
                    changed = true;
                }
            }

            if (changed)
            {
                EditorUtility.SetDirty(mat);
                fixed_count++;
                Debug.Log($"Reset tint: {mat.name}");
            }
        }

        AssetDatabase.SaveAssets();
        Debug.Log($"Done! Reset colour tint on {fixed_count}/{matGuids.Length} materials.");
        EditorUtility.DisplayDialog("Auto-Assign Textures",
            $"Reset colour tint on {fixed_count}/{matGuids.Length} materials.", "OK");
    }
}
