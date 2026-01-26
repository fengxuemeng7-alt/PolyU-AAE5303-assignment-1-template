# AAE5303 Environment Setup Report 

---

## 1. System Information

**Laptop model:**  
_[HUAWEI MateBook x]_

**CPU / RAM:**  
_[Intel(R) Core(TM) i7-10510U CPU @ 1.80GHz, 16GB RAM]_

**Host OS:**  
_[Windows 11]_

**Linux/ROS environment type:**  
_[Choose one:]_
- [ ] Dual-boot Ubuntu
- [ ] WSL2 Ubuntu
- [ ] Ubuntu in VM (UTM/VirtualBox/VMware/Parallels)
- [x] Docker container
- [ ] Lab PC
- [ ] Remote Linux server

---

## 2. Python Environment Check

### 2.1 Steps Taken

Describe briefly how you created/activated your Python environment:

**Tool used:**  
_[ system Python]_

**Key commands you ran:**
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Any deviations from the default instructions:**  
_[None]_

### 2.2 Test Results

Run these commands and paste the actual terminal output (not just screenshots):

```bash
python scripts/test_python_env.py
```

**Output:**
```
[========================================
AAE5303 Environment Check (Python + ROS)
Goal: help you verify your environment and understand what each check means.
========================================

Step 1: Environment snapshot
  Why: We capture platform/Python/ROS variables to diagnose common setup mistakes (especially mixed ROS env).
Step 2: Python version
  Why: The course assumes Python 3.10+; older versions often break package wheels.
Step 3: Python imports (required/optional)
  Why: Imports verify packages are installed and compatible with your Python version.
Step 4: NumPy sanity checks
  Why: We run a small linear algebra operation so success means more than just `import numpy`.
Step 5: SciPy sanity checks
  Why: We run a small FFT to confirm SciPy is functional (not just installed).
Step 6: Matplotlib backend check
  Why: We generate a tiny plot image (headless) to confirm plotting works on your system.
Step 7: OpenCV PNG decoding (subprocess)
  Why: PNG decoding uses native code; we isolate it so corruption/codec issues cannot crash the whole report.
Step 8: Open3D basic geometry + I/O (subprocess)
  Why: Open3D is a native extension; ABI mismatches can segfault. Subprocess isolation turns crashes into readable failures.
Step 9: ROS toolchain checks
  Why: The course requires ROS tooling. This check passes if ROS 2 OR ROS 1 is available (either one is acceptable).
  Action: building ROS 2 workspace package `env_check_pkg` (this may take 1-3 minutes on first run)...
  Action: running ROS 2 talker/listener for a few seconds to verify messages flow...
Step 10: Basic CLI availability
  Why: We confirm core commands exist on PATH so students can run the same commands as in the labs.

=== Summary ===
✅ Environment: {
  "platform": "Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.35",
  "python": "3.10.12",
  "executable": "/root/PolyU-AAE5303-env-smork-test/.venv/bin/python",
  "cwd": "/root/PolyU-AAE5303-env-smork-test",
  "ros": {
    "ROS_VERSION": "2",
    "ROS_DISTRO": "humble",
    "ROS_ROOT": null,
    "ROS_PACKAGE_PATH": null,
    "AMENT_PREFIX_PATH": "/opt/ros/humble",
    "CMAKE_PREFIX_PATH": null
  }
}
✅ Python version OK: 3.10.12
✅ Module 'numpy' found (v2.2.6).
✅ Module 'scipy' found (v1.15.3).
✅ Module 'matplotlib' found (v3.10.8).
✅ Module 'cv2' found (v4.12.0).
✅ Module 'rclpy' found (vunknown).
✅ numpy matrix multiply OK.
✅ numpy version 2.2.6 detected.
✅ scipy FFT OK.
✅ scipy version 1.15.3 detected.
✅ matplotlib backend OK (Agg), version 3.10.8.
✅ OpenCV OK (v4.12.0), decoded sample image 128x128.
✅ Open3D OK (v0.19.0), NumPy 2.2.6.
✅ Open3D loaded sample PCD with 8 pts and completed round-trip I/O.
✅ ROS 2 CLI OK: /opt/ros/humble/bin/ros2
✅ ROS 1 tools not found (acceptable if ROS 2 is installed).
✅ colcon found: /usr/bin/colcon
✅ ROS 2 workspace build OK (env_check_pkg).
✅ ROS 2 runtime OK: talker and listener exchanged messages.
✅ Binary 'python3' found at /root/PolyU-AAE5303-env-smork-test/.venv/bin/python3

All checks passed. You are ready for AAE5303 🚀]
```

```bash
python scripts/test_open3d_pointcloud.py
```

**Output:**
```
[ℹ️ Loading /root/PolyU-AAE5303-env-smork-test/data/sample_pointcloud.pcd ...
✅ Loaded 8 points.
   • Centroid: [0.025 0.025 0.025]
   • Axis-aligned bounds: min=[0. 0. 0.], max=[0.05 0.05 0.05]
✅ Filtered point cloud kept 7 points.
✅ Wrote filtered copy with 7 points to /root/PolyU-AAE5303-env-smork-test/data/sample_pointcloud_copy.pcd
   • AABB extents: [0.05 0.05 0.05]
   • OBB  extents: [0.08164966 0.07071068 0.05773503], max dim 0.0816 m
🎉 Open3D point cloud pipeline looks good.]
```

**Screenshot:**  
_[Include one screenshot showing both tests passing]_

<img width="1047" height="899" alt="屏幕截图 2026-01-18 123639" src="https://github.com/user-attachments/assets/50008668-d790-4a4f-a58c-75cdbbdb4aa9" />
<img width="1034" height="892" alt="屏幕截图 2026-01-18 123717" src="https://github.com/user-attachments/assets/05f5031c-5325-438b-833b-d1c8515dfd5b" />
<img width="1039" height="386" alt="屏幕截图 2026-01-18 123739" src="https://github.com/user-attachments/assets/78c3c027-9c5a-49d3-a900-bbdded6fd667" />


---

## 3. ROS 2 Workspace Check

### 3.1 Build the workspace

Paste the build output summary (final lines only):

```bash
source /opt/ros/humble/setup.bash
colcon build
```

**Expected output:**
```
Summary: 1 package finished [x.xx s]
```

**Your actual output:**
```
[Starting >>> env_check_pkg
Finished <<< env_check_pkg [0.58s]

Summary: 1 package finished [1.47s]]
```

### 3.2 Run talker and listener

Show both source commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

**Then run talker:**
```bash
ros2 run env_check_pkg talker.py
```

**Output (3–4 lines):**
```
[[INFO] [1768716397.127679099] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #17'
[INFO] [1768716397.629528545] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #18'
[INFO] [1768716398.131486750] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #19'
[INFO] [1768716398.633482611] [env_check_pkg_talker]: Publishing: 'AAE5303 hello #20']
```

**Run listener:**
```bash
ros2 run env_check_pkg listener.py
```

**Output (3–4 lines):**
```
[[INFO] [1768717917.335107237] [env_check_pkg_listener]: I heard: 'AAE5303 hello #73'
[INFO] [1768717917.836244849] [env_check_pkg_listener]: I heard: 'AAE5303 hello #74'
[INFO] [1768717918.337884056] [env_check_pkg_listener]: I heard: 'AAE5303 hello #75'
[INFO] [1768717918.838925070] [env_check_pkg_listener]: I heard: 'AAE5303 hello #76'
]
```

**Alternative (using launch file):**
```bash
ros2 launch env_check_pkg env_check.launch.py
```

**Screenshot:**  
_[Include one screenshot showing talker + listener running]_

![Talker and Listener Running](path/to/your/screenshot.png)

<img width="1078" height="773" alt="屏幕截图 2026-01-18 144240" src="https://github.com/user-attachments/assets/d2685bdd-9f3c-482d-94c4-ac95e7c69578" />

---

## 4. Problems Encountered and How I Solved Them

> **Note:** Write 2–3 issues, even if small. This section is crucial — it demonstrates understanding and problem-solving.

### Issue 1: [ros2: command not found]

**Cause / diagnosis:**  
_[ROS2 environment variables are not loaded in the current shell session. Although ROS2 Humble is installed on the system (located at `/opt/ros/humble/`), you need to explicitly source the setup.bash file to use the `ros2` command.]_

**Fix:**  
_[Source the ROS2 base environment before running any ros2 commands:]_

```bash
[source /opt/ros/humble/setup.bash && ros2 launch env_check_pkg env_check.launch.py]
```

**Reference:**  
_[- ROS2 Humble official installation documentation on environment configuration
- ROS2 workspace getting started tutorial]_

---

### Issue 2: [ Package 'env_check_pkg' Not Found]

**Cause / diagnosis:**  
_[Only the ROS2 base environment was sourced, but the custom workspace environment was not loaded. The `env_check_pkg` package is located at `/root/PolyU-AAE5303-env-smork-test/ros2_ws/`, and ROS2 by default only searches system-installed package paths, so it cannot find custom packages compiled and installed in the workspace.]_

**Fix:**  
_[Need to source both the ROS2 base environment and the workspace's install/setup.bash:]_

```bash
[cd /root/PolyU-AAE5303-env-smork-test/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch env_check_pkg env_check.launch.py]
```

**Reference:**  
_[- ROS2 workspace concept documentation (colcon workspace)
- Stack Overflow: "ROS2 package not found" related discussions
- Environment sourcing section in ROS2 development best practices guide]_

---

### Issue 3 (Optional): [ Multiple Shell Sessions Requiring Environment Setup]

**Cause / diagnosis:**  
_[Every time a new terminal window or shell session is opened, the environment needs to be sourced again. If not sourced, previous commands will fail again. This is a design feature of ROS2 - each new shell requires manual environment loading.]_

**Fix:**  
_[Add the source commands to the `.bashrc` file so they execute automatically when each new terminal opens, or create a startup script to conveniently set up the environment.]_

```bash
[# Option 1: Add to ~/.bashrc (permanent solution)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/PolyU-AAE5303-env-smork-test/ros2_ws/install/setup.bash" >> ~/.bashrc
# Option 2: Create a convenience script (temporary solution)
cat > ~/setup_ros2_ws.sh << 'EOF'
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/PolyU-AAE5303-env-smork-test/ros2_ws/install/setup.bash
EOF
chmod +x ~/setup_ros2_ws.sh
# Usage: source ~/setup_ros2_ws.sh]
```

**Reference:**  
_[- ROS2 environment configuration best practices
- Linux bashrc configuration file documentation
- ROS2 developer community discussions]_

---

## 5. Use of Generative AI (Required)

Choose one of the issues above and document how you used AI to solve it.

> **Goal:** Show critical use of AI, not blind copying.

### 5.1 Exact prompt you asked

**Your prompt:**
```
[ Please help me configure the Multiple Shell Sessions Requiring Environment Setup so that it doesn't require manual loading each time I open the application.]
```

### 5.2 Key helpful part of the AI's answer

**AI's response (relevant part only):**
```
[Multiple Shell Sessions Requiring Environment Setup

**Error message:**

(Not an explicit error, but a workflow issue)]
```

**### 5.3 What you changed or ignored and why**
```
Explain briefly:
- Did the AI recommend something unsafe?
- Did you modify its solution?
- Did you double-check with official docs?
```

**Your explanation:**  
_[The artificial intelligence did not recommend any unsafe content to me. I modified some of its schemes and I have also consulted relevant materials.
- Script created: `~/setup_ros2_ws.sh` contains the necessary source commands
- Script is executable: execution permissions have been set
- Functionality verified:
  - ROS2 commands are available (`ros2 --help` works correctly)
  - Workspace packages can be found (`env_check_pkg` appears in the package list)]_
```
```
**### 5.4 Final solution you applied**

Show the exact command or file edit that fixed the problem:

```bash
:source ~/setup_ros2_ws.sh
```

**Why this worked:**  
_[A convenient script ~/setup_ros2_ws.sh has been created and the verification has also been passed.
The script has been created: ~/setup_ros2_ws.sh contains the necessary source command.
The script is executable: execution permissions have been set.
Functional verification has been conducted.
ROS2 commands are available (ros2 --help works normally).
The workspace packages can be found (env_check_pkg appears in the package list). Therefore,
In each new shell session, running this command will set the environment. This can be done each time to prevent omissions.]_

---

## 6. Reflection (3–5 sentences)

Short but thoughtful:

- What did you learn about configuring robotics environments?
- What surprised you?
- What would you do differently next time (backup, partitioning, reading error logs, asking better AI questions)?
- How confident do you feel about debugging ROS/Python issues now?

**Your reflection:**

_[Through troubleshooting these ROS2 environment issues, I learned several key lessons about robotics environment configuration:

**1. Environment sourcing is mandatory, not optional**
- ROS2 requires explicit environment setup by sourcing `setup.bash` files in each shell session
- Unlike many tools that work immediately after installation, ROS2 environments must be manually loaded
- This design allows multiple ROS2 distributions to coexist on the same system

**2. Layered environment architecture**
- ROS2 uses a two-tier structure: base installation (`/opt/ros/humble/setup.bash`) + workspace (`install/setup.bash`)
- Base installation provides core ROS2 functionality
- Workspace setup extends the environment with custom packages
- Both must be sourced in the correct order for packages to be discoverable

**3. Automation is essential for productivity**
- Manual sourcing works for single sessions but becomes tedious quickly
- Creating setup scripts or `.bashrc` entries saves significant time
- Scripts offer flexibility; `.bashrc` provides persistence - choose based on project needs

**4. Error messages contain diagnostic clues**
- Messages like `"package not found, searching: ['/opt/ros/humble']"` reveal what paths ROS2 is checking
- This information directly points to missing environment sourcing steps
- Learning to read error messages carefully accelerates debugging

I am astonished by the power of current AI technology. I never expected that AI could accomplish so many things. I am truly shocked by its immense capabilities. Next time, I will pose better questions to enable the AI to assist me. I have noticed that different ways of asking questions can lead to different answer results. And make multiple backups in advance to prevent the situation of having to start all over again. With the completion of this assignment, I now have great confidence in being able to finish it.
]_

---

## 7. Declaration

✅ **I confirm that I performed this setup myself and all screenshots/logs reflect my own environment.**

**Name:**  
_[FENG XUEMENG]_

**Student ID:**  
_[25049422G]_

**Date:**  
_[26/1/2026]_

---

## Submission Checklist

Before submitting, ensure you have:

- [X] Filled in all system information
- [X] Included actual terminal outputs (not just screenshots)
- [X] Provided at least 2 screenshots (Python tests + ROS talker/listener)
- [X] Documented 2–3 real problems with solutions
- [X] Completed the AI usage section with exact prompts
- [X] Written a thoughtful reflection (3–5 sentences)
- [X] Signed the declaration

---

**End of Report**
