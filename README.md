# OpenQDD
Development of a low cost, high tourque, quasi direct drive actuator for robotics projects. Also its mostly 3D printed.  
It implements a modified cycloid disc, based on the research of Junrong Li, Ke An, Xiaozhong Deng and Jinfan Li "A New Tooth Profile Modification Method of Cycloidal Gears in Precision Reducers for Robots" [(DOI:10.3390/app10041266)](https://www.researchgate.net/publication/339282994_A_New_Tooth_Profile_Modification_Method_of_Cycloidal_Gears_in_Precision_Reducers_for_Robots). 
## Software  
To create the modified disc, do the following:
1. Open `config.py` and change the described parameters as needed
2. Set the option `create_dxf` to true
3. Run `Cycloid_plotter.py`. A `.dxf` file will be created in the same directory
4. Import the `.dxf` into your CAD package of choice

Alternative way:
1. In the `config.py` activate the option `create_csv`
2. Run `Cycloid_plotter.py`. A `.csv` file will be created in the same directory
3. Import the `.dxf` into your CAD package of choice and create a spline. For Fusion360 i recommend an [alternate importer](https://github.com/CADstudioCZ/ImportSplineCSVg/) as the standard one has a few issues. 
This may lead to a few issues at the start/end of the spline, so dxf is the prefered option.
