# OpenQDD
Development of a low cost, high tourque, quasi direct drive actuator for robotics projects. Also its mostly 3D printed.  
It implements a modified cycloid disc, based on the research of Junrong Li, Ke An, Xiaozhong Deng and Jinfan Li "A New Tooth Profile Modification Method of Cycloidal Gears in Precision Reducers for Robots" [(DOI:10.3390/app10041266)](https://www.researchgate.net/publication/339282994_A_New_Tooth_Profile_Modification_Method_of_Cycloidal_Gears_in_Precision_Reducers_for_Robots). 
## Software  
To create the modified disc, do the following:
1. Open `config.py` and change the described parameters
2. Run `Cycloid_plotter.py`. A `.csv` file will be created in the same directory
3. Import the `.csv` into your CAD package of choice and create a spline. For Fusion360 i recommend an [alternate importer](https://github.com/CADstudioCZ/ImportSplineCSVg/) as the standard one has a few issues. 
