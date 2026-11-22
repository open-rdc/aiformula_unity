using GaussianSplatting.Runtime;
using UnityEditor;
using UnityEngine;

public static class GaussianWorldSetup
{
    const string AssetPath = "Assets/GaussianSplats/point_cloud (1).asset";
    static readonly Vector3 LevelEuler = new Vector3(55.12f, 176.81f, 178.33f);
    static readonly Vector3 LevelOffset = new Vector3(1.014f, 1.915f, 0.592f);

    [MenuItem("Tools/Gaussian Splat/Add point_cloud (1) (ground aligned)")]
    static void Add()
    {
        var asset = AssetDatabase.LoadAssetAtPath<GaussianSplatAsset>(AssetPath);
        if (asset == null)
        {
            EditorUtility.DisplayDialog("Gaussian Splat", $"{AssetPath} が見つかりません", "OK");
            return;
        }

        var world = new GameObject("GaussianWorld");
        Undo.RegisterCreatedObjectUndo(world, "Add Gaussian World");

        var splat = new GameObject(asset.name);
        splat.SetActive(false);
        splat.transform.SetParent(world.transform, false);
        splat.transform.localRotation = Quaternion.Euler(LevelEuler);
        splat.transform.localPosition = LevelOffset;

        var r = splat.AddComponent<GaussianSplatRenderer>();
        r.m_Asset = asset;
        r.m_ShaderSplats       = Load<Shader>("ed800126ae8844a67aad1974ddddd59c");
        r.m_ShaderComposite    = Load<Shader>("7e184af7d01193a408eb916d8acafff9");
        r.m_ShaderDebugPoints  = Load<Shader>("b44409fc67214394f8f47e4e2648425e");
        r.m_ShaderDebugBoxes   = Load<Shader>("4006f2680fd7c8b4cbcb881454c782be");
        r.m_CSSplatUtilities   = Load<ComputeShader>("ec84f78b836bd4f96a105d6b804f08bd");
        splat.SetActive(true);

        Undo.RegisterCreatedObjectUndo(splat, "Add Gaussian Splat");
        Selection.activeGameObject = world;
        Debug.Log($"[GaussianWorldSetup] {asset.name} ({asset.splatCount} splats) を追加しました。" +
                  "GaussianWorld の Position / Rotation Y / Scale でコースに合わせてください");
    }

    [MenuItem("Tools/Gaussian Splat/Add movie1 (same as gaussian_simlator)")]
    static void AddMovie1LikeGaussianSimulator()
    {
        const string path = "Assets/GaussianAssets/movie1.asset";
        var asset = AssetDatabase.LoadAssetAtPath<GaussianSplatAsset>(path);
        if (asset == null)
        {
            EditorUtility.DisplayDialog("Gaussian Splat", $"{path} が見つかりません", "OK");
            return;
        }
        var go = new GameObject("Gaussian");
        go.SetActive(false);
        go.transform.localPosition = new Vector3(0.02f, 0.4f, 0.15f);
        go.transform.localRotation = new Quaternion(-0.0029723528f, 0.107324034f, -0.9933322f, -0.04199918f);
        go.transform.localScale = new Vector3(-1f, 1f, 1f);
        Setup(go.AddComponent<GaussianSplatRenderer>(), asset);
        go.SetActive(true);
        Undo.RegisterCreatedObjectUndo(go, "Add Gaussian (movie1)");
        Selection.activeGameObject = go;
        Debug.Log("[GaussianWorldSetup] movie1 を gaussian_simlator と同じ配置で追加しました");
    }

    static void Setup(GaussianSplatRenderer r, GaussianSplatAsset asset)
    {
        r.m_Asset = asset;
        r.m_ShaderSplats       = Load<Shader>("ed800126ae8844a67aad1974ddddd59c");
        r.m_ShaderComposite    = Load<Shader>("7e184af7d01193a408eb916d8acafff9");
        r.m_ShaderDebugPoints  = Load<Shader>("b44409fc67214394f8f47e4e2648425e");
        r.m_ShaderDebugBoxes   = Load<Shader>("4006f2680fd7c8b4cbcb881454c782be");
        r.m_CSSplatUtilities   = Load<ComputeShader>("ec84f78b836bd4f96a105d6b804f08bd");
    }

    static T Load<T>(string guid) where T : Object =>
        AssetDatabase.LoadAssetAtPath<T>(AssetDatabase.GUIDToAssetPath(guid));
}
