using UnityEngine;
using UnityEditor;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// One-click setup for all HoloAssist UI features in the current scene.
/// Use: GameObject menu → HoloAssist → Setup All UI Features
/// Creates panels, radial menu, bin detectors, TF visualizer, passthrough toggle,
/// virtual environment, and wires references automatically.
/// </summary>
public static class HoloAssistSetup
{
    [MenuItem("Tools/HoloAssist/Setup All UI Features")]
    public static void SetupAll()
    {
        Debug.Log("[HoloAssistSetup] Starting auto-setup...");

        var robotRoot = FindRobotRoot();
        var controller = FindControllerObject();
        var leftCtrl = FindController("left");
        var rightCtrl = FindController("right");
        var baseLink = FindByName("base_link");
        var tool0 = FindByName("tool0");

        // 1. Add JointTFVisualizer to robot root
        var tfVis = SetupTFVisualizer(robotRoot);

        // 2. Create the four panels
        var dataPanel = CreatePanel<RobotDataPanel>("RobotDataPanel", new Vector3(-0.5f, 1.5f, 1.5f));
        var binPanel = CreatePanel<BinStatusPanel>("BinStatusPanel", new Vector3(0.5f, 1.5f, 1.5f));
        var coachPanel = CreatePanel<CoachingPanel>("CoachingPanel", new Vector3(0f, 1.7f, 1.6f));
        coachPanel.gameObject.SetActive(false); // hidden by default

        // 3. Create RadialMenu (reuse if exists)
        var radial = Object.FindFirstObjectByType<RadialMenu>();
        if (radial == null)
        {
            var radialGO = new GameObject("RadialMenu");
            radial = radialGO.AddComponent<RadialMenu>();
        }
        else Debug.Log("[HoloAssistSetup] RadialMenu already exists — reusing");

        // 4. Create PassthroughToggle (reuse if exists)
        var pt = Object.FindFirstObjectByType<PassthroughToggle>();
        if (pt == null)
        {
            var ptGO = new GameObject("PassthroughToggle");
            pt = ptGO.AddComponent<PassthroughToggle>();
        }
        else Debug.Log("[HoloAssistSetup] PassthroughToggle already exists — reusing");

        // 5. Create VirtualEnvironment with ground plane (reuse if exists)
        var virtEnv = GameObject.Find("VirtualEnvironment");
        if (virtEnv == null)
        {
            virtEnv = new GameObject("VirtualEnvironment");

            // Ground plane (visible mesh)
            var ground = GameObject.CreatePrimitive(PrimitiveType.Plane);
            ground.name = "VRGround";
            ground.transform.SetParent(virtEnv.transform, false);
            ground.transform.localScale = new Vector3(10, 1, 10);

            var groundMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/GroundMat.mat");
            if (groundMat != null) ground.GetComponent<Renderer>().material = groundMat;

            // Add a thick BoxCollider as a safety net in case objects sneak past the plane
            var safety = new GameObject("GroundSafety");
            safety.transform.SetParent(virtEnv.transform, false);
            safety.transform.localPosition = new Vector3(0, -0.05f, 0);
            var box = safety.AddComponent<BoxCollider>();
            box.size = new Vector3(20f, 0.1f, 20f);

            // Keep environment active so colliders work even when renderers are off
            // (PassthroughToggle.keepCollidersInPassthrough handles renderer toggle at runtime)
            // Renderers stay ENABLED in edit mode so you can see the ground while setting up.
            virtEnv.SetActive(true);
        }
        else Debug.Log("[HoloAssistSetup] VirtualEnvironment already exists — reusing");
        pt.virtualEnvironment = virtEnv;
        pt.vrSkyboxMaterial = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/VRSkybox.mat");

        // 6. Wire RobotDataPanel
        if (controller != null)
            dataPanel.robotController = controller.GetComponent<RobotController>();
        if (tool0 != null) dataPanel.endEffectorTransform = tool0.transform;
        if (baseLink != null) dataPanel.robotBase = baseLink.transform;
        if (tfVis != null) dataPanel.tfVisualizer = tfVis;

        // 7. Wire RadialMenu
        radial.tfVisualizer = tfVis;
        radial.dataPanel = dataPanel;
        radial.binStatusPanel = binPanel;
        radial.coachingPanel = coachPanel;
        radial.passthroughToggle = pt;
        if (controller != null) radial.robotController = controller.GetComponent<RobotController>();

        // Find existing RobotHUD in scene and add PanelPlacer + wire to radial menu
        var hud = Object.FindFirstObjectByType<RobotHUD>();
        if (hud != null)
        {
            radial.robotHUD = hud;
            // Wire all references so HUD mirrors the radial menu state
            if (controller != null) hud.controller = controller.GetComponent<RobotController>();
            hud.tfVisualizer = tfVis;
            hud.passthroughToggle = pt;
            AddPanelPlacer(hud.gameObject, leftCtrl, rightCtrl, 0.55f, 0.14f);
        }
        if (leftCtrl != null) radial.leftControllerOverride = leftCtrl.transform;
        if (rightCtrl != null) radial.rightControllerOverride = rightCtrl.transform;

        // 8. Add PanelPlacer to each panel
        AddPanelPlacer(dataPanel.gameObject, leftCtrl, rightCtrl, 0.8f, 0.75f);
        AddPanelPlacer(binPanel.gameObject, leftCtrl, rightCtrl, 0.5f, 0.3f);
        AddPanelPlacer(coachPanel.gameObject, leftCtrl, rightCtrl, 0.7f, 0.5f);

        // 9. Find bins in scene and add BinDetector
        SetupBins(binPanel);

        // 10. (Skipped — Nic's RobotController doesn't expose tfVisualizer field;
        //     TF toggle works via radial menu's "TF Axes" button instead of X button)

        // 11. Coaching right controller
        if (rightCtrl != null) coachPanel.rightControllerOverride = rightCtrl.transform;

        // 12. Create grabbable Cube with ROSObjectPublisher
        var cube = SetupCube(tfVis, baseLink);

        // 13. Link cube to JointTFVisualizer's linkedPublishers
        if (tfVis != null && cube != null)
        {
            var pub = cube.GetComponent<ROSObjectPublisher>();
            if (pub != null)
            {
                tfVis.linkedPublishers = new ROSObjectPublisher[] { pub };
            }
        }

        // Mark scene dirty so Unity saves it
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());

        Debug.Log("[HoloAssistSetup] Done! Save the scene (Ctrl+S) to keep changes.");
        EditorUtility.DisplayDialog("HoloAssist Setup",
            "Setup complete!\n\nRemember to:\n" +
            "• Save the scene (Ctrl+S)\n" +
            "• Position the bins where you want them\n" +
            "• Verify references in Inspector if anything is missing",
            "OK");
    }

    static GameObject FindRobotRoot()
    {
        // Try common names
        foreach (var name in new[] { "ur3e_robot", "ur", "ur3e", "Robot" })
        {
            var go = GameObject.Find(name);
            if (go != null) return go;
        }
        Debug.LogWarning("[HoloAssistSetup] Could not find robot root — assign JointTFVisualizer manually");
        return null;
    }

    static GameObject FindControllerObject()
    {
        // GameObject named "controller" with RobotController component
        var all = Object.FindObjectsByType<RobotController>(FindObjectsSortMode.None);
        if (all.Length > 0) return all[0].gameObject;
        return null;
    }

    static GameObject FindController(string side)
    {
        // Look for common controller GameObject names
        string[] candidates = side == "left"
            ? new[] { "Left Controller", "LeftHand Controller", "Left Hand", "XR Left Controller", "LeftHand" }
            : new[] { "Right Controller", "RightHand Controller", "Right Hand", "XR Right Controller", "RightHand" };
        foreach (var n in candidates)
        {
            var go = GameObject.Find(n);
            if (go != null) return go;
        }

        // Fallback: search all transforms (including inactive) for matching name
        var all = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        foreach (var t in all)
        {
            string lname = t.name.ToLower();
            if (lname.Contains(side) && lname.Contains("controller"))
                return t.gameObject;
        }
        return null;
    }

    static GameObject FindByName(string name)
    {
        var go = GameObject.Find(name);
        if (go != null) return go;

        // Search inactive too
        var allTransforms = Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        foreach (var t in allTransforms)
            if (t.name == name) return t.gameObject;
        return null;
    }

    static JointTFVisualizer SetupTFVisualizer(GameObject robotRoot)
    {
        if (robotRoot == null) return null;
        var existing = robotRoot.GetComponent<JointTFVisualizer>();
        if (existing != null) return existing;
        return robotRoot.AddComponent<JointTFVisualizer>();
    }

    static T CreatePanel<T>(string name, Vector3 pos) where T : Component
    {
        // Avoid duplicates
        var existing = Object.FindFirstObjectByType<T>();
        if (existing != null)
        {
            Debug.Log($"[HoloAssistSetup] {name} already exists — reusing");
            return existing;
        }

        var go = new GameObject(name);
        go.transform.position = pos;
        return go.AddComponent<T>();
    }

    static void AddPanelPlacer(GameObject panel, GameObject leftCtrl, GameObject rightCtrl,
        float w, float h)
    {
        if (panel.GetComponent<PanelPlacer>() != null) return;
        var pp = panel.AddComponent<PanelPlacer>();
        pp.colliderWidth = w;
        pp.colliderHeight = h;
    }

    [MenuItem("Tools/HoloAssist/Setup Robot HUD")]
    public static void SetupRobotHUDOnly()
    {
        var hud = Object.FindFirstObjectByType<RobotHUD>();
        if (hud == null)
        {
            // Create new GameObject if no RobotHUD exists
            var hudGO = new GameObject("RobotHUD");
            hud = hudGO.AddComponent<RobotHUD>();
        }

        // Wire RobotController reference
        var controller = FindControllerObject();
        if (controller != null)
            hud.controller = controller.GetComponent<RobotController>();

        // Add Rigidbody for XRGrabInteractable (kinematic so it doesn't fall)
        var rb = hud.gameObject.GetComponent<Rigidbody>();
        if (rb == null) rb = hud.gameObject.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.isKinematic = true;

        // Add PanelPlacer for grabbing
        var leftCtrl = FindController("left");
        var rightCtrl = FindController("right");
        AddPanelPlacer(hud.gameObject, leftCtrl, rightCtrl, 0.55f, 0.14f);

        // Wire to RadialMenu so it can be toggled from the menu
        var radial = Object.FindFirstObjectByType<RadialMenu>();
        if (radial != null) radial.robotHUD = hud;

        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());

        Debug.Log("[HoloAssistSetup] RobotHUD setup complete. Save scene with Ctrl+S.");
        EditorUtility.DisplayDialog("HoloAssist",
            "RobotHUD configured:\n" +
            "• Movable via right-trigger ray grab\n" +
            "• Toggleable via radial menu (Robot HUD button)\n" +
            "• Styled to match other panels\n\n" +
            "Save scene with Ctrl+S.", "OK");
    }

    [MenuItem("Tools/HoloAssist/Add Grabbable Cube")]
    public static void AddCubeOnly()
    {
        var baseLink = FindByName("base_link");
        var tfVis = Object.FindFirstObjectByType<JointTFVisualizer>();

        var cube = SetupCube(tfVis, baseLink);

        // Link to JointTFVisualizer if not already
        if (tfVis != null && cube != null)
        {
            var pub = cube.GetComponent<ROSObjectPublisher>();
            if (pub != null)
            {
                bool alreadyLinked = false;
                if (tfVis.linkedPublishers != null)
                {
                    foreach (var p in tfVis.linkedPublishers)
                        if (p == pub) { alreadyLinked = true; break; }
                }
                if (!alreadyLinked)
                {
                    var newList = new System.Collections.Generic.List<ROSObjectPublisher>();
                    if (tfVis.linkedPublishers != null) newList.AddRange(tfVis.linkedPublishers);
                    newList.Add(pub);
                    tfVis.linkedPublishers = newList.ToArray();
                }
            }
        }

        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());

        Debug.Log("[HoloAssistSetup] Cube added. Save the scene (Ctrl+S).");
        EditorUtility.DisplayDialog("HoloAssist", "Cube added. Save scene with Ctrl+S.", "OK");
    }

    static GameObject SetupCube(JointTFVisualizer tfVis, GameObject baseLink)
    {
        // Reuse existing Cube if it has ROSObjectPublisher
        var existing = GameObject.Find("Cube");
        if (existing != null && existing.GetComponent<ROSObjectPublisher>() != null)
        {
            Debug.Log("[HoloAssistSetup] Cube with ROSObjectPublisher already exists — reusing");
            return existing;
        }

        // Create or reuse plain Cube GameObject
        GameObject cube = existing;
        if (cube == null)
        {
            cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.name = "Cube";
            cube.transform.position = new Vector3(0f, 1f, 1f);
            cube.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
        }

        // Rigidbody for physics
        var rb = cube.GetComponent<Rigidbody>();
        if (rb == null) rb = cube.AddComponent<Rigidbody>();
        rb.useGravity = true;
        rb.isKinematic = false;

        // XR Grab Interactable
        if (cube.GetComponent<XRGrabInteractable>() == null)
            cube.AddComponent<XRGrabInteractable>();

        // ROSObjectPublisher
        var pub = cube.GetComponent<ROSObjectPublisher>();
        if (pub == null) pub = cube.AddComponent<ROSObjectPublisher>();
        pub.frameName = "unity_cube";
        pub.parentFrame = "base_link";
        if (baseLink != null) pub.robotBase = baseLink.transform;
        pub.publishMarker = true;
        pub.showPoseAxes = true;

        // Try to auto-assign RGB axis materials from MRTemplate
        var xMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/MRTemplateAssets/Materials/AR/XMaterial.mat");
        var yMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/MRTemplateAssets/Materials/AR/YMaterial.mat");
        var zMat = AssetDatabase.LoadAssetAtPath<Material>("Assets/MRTemplateAssets/Materials/AR/ZMaterial.mat");
        if (xMat != null) pub.xMaterial = xMat;
        if (yMat != null) pub.yMaterial = yMat;
        if (zMat != null) pub.zMaterial = zMat;

        Debug.Log("[HoloAssistSetup] Cube setup: Rigidbody + XRGrabInteractable + ROSObjectPublisher");
        return cube;
    }

    static void SetupBins(BinStatusPanel panel)
    {
        // Find bins by name (Object_Bin_v1...)
        var allObjects = Object.FindObjectsByType<MeshRenderer>(FindObjectsSortMode.None);
        int binCount = 0;
        foreach (var r in allObjects)
        {
            if (!r.gameObject.name.ToLower().Contains("bin")) continue;
            if (r.GetComponent<BinDetector>() != null) continue; // already has one

            var detector = r.gameObject.AddComponent<BinDetector>();
            detector.binName = $"Bin {(char)('A' + binCount)}";
            detector.statusPanel = panel;
            binCount++;
            Debug.Log($"[HoloAssistSetup] Added BinDetector to {r.gameObject.name} as '{detector.binName}'");
        }

        if (binCount == 0)
            Debug.LogWarning("[HoloAssistSetup] No bins found in scene — drag bin FBX into scene then re-run setup or add BinDetector manually");
    }
}
