## Welcome to the UIUC Hand SDK

👉 **More info:** [UIUC Hand Website](http://uiuchand.com/)

---

### Software Setup
- See these folders for setup details:  
  - [Python API](https://github.com/your-username/UIUC_Hand_API/tree/main/python)  
  - [Useful Tools](https://github.com/your-username/UIUC_Hand_API/tree/main/useful_tools)

---

### 🔌 Hardware Setup
- Connect **5 V power** to the hand (Dynamixels should light up on boot).
- Connect the **Micro‑USB** cable (avoid multiple USB extensions).
- Use [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/rplus1/dynamixel_wizard/) to find the correct port.  
  ➡️ Put that port into `main.py` or `ros_example.py`.  
  ⚠️ You **cannot** have Dynamixel Wizard open while using the API (the port will be busy).
- On Ubuntu, find the hand by ID at `/dev/serial/by-id` (persistent across reboots).
- `sudo chmod 666 /dev/serial/by-id/(your_id)` to give serial port permissions.
- Official support: **Python** and **C++** and **ROS/ROS2**. 
  Other languages can use the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/).
- To improve latency on Ubuntu:  
  - [Adjust USB Latency Settings](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)  
  - Tune the [Dynamixel Python SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/288)  
  - Set *Return Delay Time* (Control Table Register 9) from 250 µs to **0 µs**.
- If you are using the full hand, you can raise the current limit from 300 mA to 550 mA in the API for increased strength!
---

### 🤖 Functionality
- UIUC Node allows commanding joint angles in different scalings.
- You can read **position, velocity, and current**.
- **Query limits:**  
  - Position only: ≤ 500 Hz  
  - Position + velocity + current: ≤ 500 Hz  
  (Higher rates can slow USB communication.)
- Default control: **PID** (up to current limit).  
  Velocity and current control also supported—see the [motor manual](https://emanual.robotis.com/docs/en/dxl/x/xc330-m288/).
- Current limits:  
  - Lite: ≈ 300 mA  
  - Full: up to ≈ 550 mA
  - **By default the API is at 300, you can raise it to 550mA on the Full hand!!!**
- Jittery hand? ➡️ Lower P/D values.  
  Weak hand? ➡️ Raise P/D values.

---

### 🛠️ Troubleshooting
- Motor off by 90°/180°/270° → **Remount the horn.**
- No motors show up → Check **serial port permissions**.
- Some motors missing → Verify **IDs** and **U2D2 connections**.
- Overload error (motors flashing red) → **Power cycle**. If frequent, **lower current limits**.
- Jittery motors → Lower P/D values.
- Inaccurate motors → Raise P/D values.

---

### 🔧 Useful Tools
- **MANO → UIUC** joint angle mapping.
- [Bimanual Dexterity for Complex Tasks](https://bidex-teleop.github.io/) shows how to use **Manus gloves** with UIUC Hand.
- Have a useful tool to share? **Pull requests welcome!**  
  (Or ask and I can add tools for you.)

---

### Support
- Questions/issues: **kshaw2@andrew.cmu.edu**
- **License:**  
  - Code: MIT License  
  - CAD: CC BY‑NC‑SA (non‑commercial use with attribution)
- Provided **as‑is**, without warranty.

**If you use UIUC Hand in research, please cite:**
```bibtex
@article{shaw2023uiuchand,
  title={UIUC Hand: Low-Cost, Efficient, and Anthropomorphic Hand for Robot Learning},
  author={Shaw, Kenneth and Agarwal, Ananye and Pathak, Deepak},
  journal={Robotics: Science and Systems (RSS)},
  year={2023}
}
