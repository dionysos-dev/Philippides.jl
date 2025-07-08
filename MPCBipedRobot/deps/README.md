# 🤖 URDF Models for Planar Bipedal Robot

This folder contains various URDF models usable for simulation and control of our planar bipedal robot. The models are derived from CAD assemblies designed in **Onshape** and organized into two categories:

- **Box-based models**: Simplified representations using rectangular approximations for faster symbolic computations.
- **Geometric models**: Detailed 3D models exported from Onshape assemblies.

This README explains how to export a robot from Onshape to a URDF format and post-process it for use in our simulations.

---

## 🛠️ From Onshape to URDF

A complete Onshape model of the robot is available at:  
👉 _[[Full Robot](https://cad.onshape.com/documents/002729f0ad708412f7e93c74/w/362e45f2c0db1f02f35fc58e/e/c801ee403e40dab0d2089eb1?renderMode=0&uiState=686d4ce9a26c6e69cfa48d86)]_ 

To simplify the URDF and improve runtime performance:
- All **fixed assemblies** were merged into composite parts.
- Each **moving link** (including screws, PLA parts, etc.) was modeled as a **single rigid body** to reduce dimensionality and enhance computational speed.

The resulting Onshape model of the robot is available at: 
👉 _[[Composite Robot](https://cad.onshape.com/documents/a0f70d4c5d7e2df9af20ba50/w/3c9ba49dfb8dc589ead70945/e/cd3eef1c05fb6c93b71ac3a2?renderMode=0&uiState=686d4b72170a7c62e0641adf)]_ 

⚠️ **Note on joint alignment**:  
Onshape enforces revolute joints with their axis along the **z-axis**. Since this constraint is not always directly applicable, we export the full model to URDF and then manually correct joint alignments to ensure **zero relative rotation** between connected joint frames.

---

## 📄 Step-by-Step: Export Onshape Model to URDF

### 1. Go to the `Enhanced_URDF/` directory

### 2. Create a Python virtual environment (optional but recommended)

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Create a directory for the URDF output

```bash
mkdir my_URDF
```

### 5. Configure the export

Edit the `config.json` file:

```json
{
    "documentId": "a0f70d4c5d7e2df9af20ba50",
    "outputFormat": "urdf",
    "assemblyName": "Composite_Robot"
}
```

### 6. Run the exporter

```bash
onshape-to-robot my_URDF/
```

---

## 🧩 Manual Post-Processing

Once the URDF is generated, you will need to:

- Manually adjust joint frames so that **no relative rotation** exists between connected frames.
- Add the robot’s **boom** manually (if required by your simulation).

---

## ✅ Ready-to-Use Models

Two finalized URDF models are available and ready for simulation:

- `composite_piece/planar_locked_biped_robot/` – For locked base simulations.
- `composite_piece/biped_robot/` – For floating base or boom-attached scenarios.

---

## 📎 Notes

- Use the simplified box model when symbolic computation speed is critical.
- Use the geometric model for visualizations and more accurate collision/contact modeling.

---

**Developed as part of the Dionysos Project – 2025**
