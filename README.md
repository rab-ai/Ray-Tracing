# Ray Tracing Implementation  

This project is a ray tracing application developed as part of the **CENG 477: Introduction to Computer Graphics** course at Middle East Technical University. It simulates the propagation of light to render realistic 3D scenes using geometric optics and shading techniques.  

## Project Description  
The application is capable of:  
- Parsing XML scene files to configure objects, lights, and cameras.  
- Rendering scenes with **Blinn-Phong shading** for realistic lighting effects.  
- Supporting point and ambient light sources, with accurate falloff calculations for point lights.  
- Handling recursive reflections for mirror-like surfaces, with adjustable recursion depth.  
- Rendering scenes from multiple camera configurations and saving outputs in PPM image format.  

## Key Features  
- **Ray-Object Intersection**: Calculates intersections between rays and scene objects efficiently.  
- **Blinn-Phong Shading**: Implements a lighting model that combines ambient, diffuse, and specular reflections.  
- **Recursive Reflections**: Traces rays bouncing off reflective surfaces for realistic mirror effects.  
- XML-based configuration for easy customization of scenes.  

## How to Run  
1. Compile the program using the provided `Makefile`:  
   ```bash
   make
2. Run the executable with:  
   ```bash
   ./raytracer scene.xml
3. The rendered images will be saved in the filenames specified in the XML file.
