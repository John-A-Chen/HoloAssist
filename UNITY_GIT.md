# Unity Git Collaboration Guide

Common git problems that arise when multiple people work on the same Unity project and how to fix them.

---

## 1. Conflict markers committed in Unity asset files

### What it looks like

Unity opens with a flood of errors like:
```
The file 'Assets/Materials/TrolleyMat.mat' seems to have merge conflicts.
Please open it in a text editor and fix the merge.
```

Running `grep -rl "<<<<<<<" Assets/` shows many `.mat`, `.unity`, `.asset`, and `.meta` files.

### Why it happens

Unity stores almost everything as YAML text files (scenes, materials, prefabs, settings). When two people independently create an asset of the same type — say, a new material — Unity uses the same default internal fileID (e.g. `&2100000` for every material) inside that file. Git's 3-way merge sees two files that both contain `&2100000` and tries to merge their content line-by-line. This causes two completely unrelated assets to be merged together, producing nonsense output like `TrolleyMat.mat` containing the name `SIGNAGE_SIGNAGE_BORDER`.

The same problem affects `SampleScene.unity` heavily: any save in Unity reorders all scene objects in the YAML, so even trivial scene changes produce thousands of lines of diff and huge conflicts.

The issue becomes a committed problem when someone runs `git commit` without resolving the markers first. The file is committed with `<<<<<<<`, `=======`, `>>>>>>>` markers still in it, and Unity can no longer parse it.

### How to detect

```bash
grep -rl "<<<<<<" "Unity/My project/Assets/"
```

### How to fix

For each conflicted file, decide which version to keep — **ours** (HEAD) or **theirs** (the other branch). The rule of thumb:

| File location | Keep |
|---|---|
| `Assets/Materials/YourMaterial.mat` | **ours** — keep the material that belongs in that file |
| `Assets/PortalMesh/.../source/materials/*.mat` | **theirs** — keep the environment material content |
| `Assets/Samples/` (XR packages) | **ours** — package-managed, don't accept external changes |
| `Assets/Scenes/SampleScene.unity` | **ours** (scene owner's version) — see Section 3 |
| `.meta` files for your own assets | **ours** — preserves the GUIDs your scene uses |
| `.meta` files for the other branch's new assets | **theirs** — uses the GUIDs that branch's scene references |

Use this Python script to resolve all conflicts at once. Adjust the `files` list to match your actual conflict list:

```python
#!/usr/bin/env python3
import re, os

def resolve(content, keep='ours'):
    result, state = [], 'normal'
    for line in content.split('\n'):
        if re.match(r'^<{7}', line):   state = 'ours'
        elif re.match(r'^={7}', line) and state == 'ours':   state = 'theirs'
        elif re.match(r'^>{7}', line) and state == 'theirs': state = 'normal'
        elif state == 'normal': result.append(line)
        elif state == 'ours'   and keep == 'ours':   result.append(line)
        elif state == 'theirs' and keep == 'theirs': result.append(line)
    return '\n'.join(result)

base = "Unity/My project"
files = [
    # ("path/relative/to/base", "ours" or "theirs"),
    ("Assets/Materials/TrolleyMat.mat",    "ours"),
    ("Assets/Scenes/SampleScene.unity",    "ours"),
    # ... add all files from grep output above
]
for rel, keep in files:
    path = os.path.join(base, rel)
    content = open(path, encoding='utf-8').read()
    if '<<<<<<' not in content:
        print(f"skip  {rel}"); continue
    open(path, 'w', encoding='utf-8').write(resolve(content, keep))
    print(f"fixed [{keep}] {rel}")
```

After running the script, verify with:
```bash
grep -rl "<<<<<<" "Unity/My project/Assets/"   # should return nothing
```

Then let Unity reimport (it will detect the file changes automatically when you switch focus to the Editor).

### Long-term prevention

Add a `.gitattributes` file that tells git to always prefer one side for Unity YAML files rather than trying to merge them:

```
# Unity YAML — always take ours on conflict; resolve manually when needed
Unity/My project/Assets/**/*.unity  merge=ours
Unity/My project/Assets/**/*.prefab merge=ours
Unity/My project/Assets/**/*.mat    merge=ours
Unity/My project/Assets/**/*.asset  merge=ours
Unity/My project/Assets/**/*.meta   merge=ours
```

Or, install **UnityYAMLMerge** (ships with Unity at `Unity/Editor/Data/Tools/UnityYAMLMerge`) and configure git to use it as a merge driver — it understands Unity's YAML semantics and resolves non-conflicting component changes automatically.

---

## 2. Scene file ownership

`SampleScene.unity` is a single file edited by everyone. Every Unity save reorders the entire YAML, producing massive diffs that are impossible to review and nearly impossible to merge correctly.

**Recommended workflow:**

- Designate one person as the **scene owner** per sprint. Only they commit scene changes.
- Everyone else works in separate scenes or prefabs and the scene owner integrates.
- When merging, always take the scene owner's version (`merge=ours` or `git checkout --ours`).
- Commit the scene file last, after all script and prefab changes are in.

If two people must both change the scene, coordinate via a call and do it synchronously — one person shares screen, the other narrates changes.

---

## 3. Build outputs in git (APK files)

Unity APK builds are large binaries and must not be committed. The `.gitignore` already contains:

```
Unity/My project/*.apk
```

However, **gitignore only affects untracked files**. If an APK was ever committed it will keep appearing in `git status` even after adding the ignore rule. To stop tracking it:

```bash
git rm --cached "Unity/My project/xrtest.apk"
git commit -m "Stop tracking build APK"
```

Never commit build outputs. Build locally, deploy via `adb install`, discard the APK.

---

## 4. Diverged remote — force push

When the remote `main` has commits that are not in your local history (e.g. a teammate pushed directly to `main`), `git push` will be rejected with:

```
! [rejected] main -> main (non-fast-forward)
```

Options, in order of preference:

1. **Pull and merge first** (safest):
   ```bash
   git pull origin main
   # resolve any conflicts, then push
   git push origin main
   ```

2. **Rebase your work on top of theirs**:
   ```bash
   git fetch origin
   git rebase origin/main
   git push origin main
   ```

3. **Force push** (destructive — only if you are certain the remote commits can be discarded):
   ```bash
   git push --force origin main
   ```
   This overwrites the remote. Anyone who already pulled those commits will have a diverged history. Only do this when everyone on the team agrees.

---

## 5. External packages need patching after a fresh clone

Two packages are checked in as local subfolders (`ROS-TCP-Connector/`, `URDF-Importer/`) and are in `.gitignore` — they must be cloned separately. After cloning them, `ROS-TCP-Connector` requires a one-line patch or Unity will stop receiving `/joint_states` after a ROS restart. See the **Required patch** section in `SETUP.md` for the exact change.

---

## Quick reference

```bash
# Check for committed conflict markers
grep -rl "<<<<<<" "Unity/My project/Assets/"

# Stop tracking a committed file that should be ignored
git rm --cached "Unity/My project/path/to/file"

# Discard all local Unity scene changes and take the remote version
git checkout origin/main -- "Unity/My project/Assets/Scenes/SampleScene.unity"

# Show which commits touched a specific Unity asset
git log --oneline -- "Unity/My project/Assets/Scenes/SampleScene.unity"
```
