![Icon

Description automatically generated](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.001.png)















![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.002.png)![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.003.png)![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.004.png)![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.005.png)










![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.006.png)![](Aspose.Words.4e129b4f-9cf0-424b-b858-a2fd4cab5531.007.png)

# Table of Contents
[1 Introduction and Initial Analysis	2](#_toc149728741)

[1.1 Project Context	2](#_toc149728742)

[1.2 Initial Thought Process	2](#_toc149728743)

[2 Requirement Analysis and Specification	2](#_toc149728744)

[2.1 User Requirements	2](#_toc149728745)

[2.2 System Requirements	2](#_toc149728746)

[2.3 Tools and Technologies	2](#_toc149728747)

[3 Conceptual Design	2](#_toc149728748)

[3.1 System Architecture	2](#_toc149728749)

[3.2 Interface Design	2](#_toc149728750)

[3.3 Control Algorithm Design	3](#_toc149728751)

[4 Detailed Design and Development	3](#_toc149728752)

[4.1 Component Design	3](#_toc149728753)

[4.2 Coding and Implementation	3](#_toc149728754)

[4.3 Integration	3](#_toc149728755)

[4.4 Unique Features	3](#_toc149728756)

[5 Testing, Evaluation, and Optimization	3](#_toc149728757)

[5.1 Testing Strategy	3](#_toc149728758)

[5.2 Performance Evaluation	3](#_toc149728759)

[5.3 Optimization	3](#_toc149728760)

[6 Collaboration and Project Management	3](#_toc149728761)

[6.1 Teamwork Dynamics	3](#_toc149728762)

[6.2 Project Management	3](#_toc149728763)

[7 Conclusion and Reflection	3](#_toc149728764)

[7.1 Project Summary	3](#_toc149728765)

[7.2 Future Work	3](#_toc149728766)

[7.3 Personal and Group Reflections	3](#_toc149728767)

[8 Appendices	4](#_toc149728768)

[8.1 Bill of Materials	4](#_toc149728769)

[8.2 Electrical Wiring and System Layout	4](#_toc149728770)

[8.3 Code Repository	4](#_toc149728771)

[8.4 Additional Documentation	4](#_toc149728772)

[9 References	4](#_toc149728773)











<a name="_toc149728741"></a>
# 1 Introduction and Initial Analysis
## <a name="_toc149728742"></a>1.1 Project Context
## <a name="_toc149728743"></a>The development of DDMR robots has been the focus of research since decades ago. The challenge in the development of this robot is to create a motion mechanism from two motors (left and right motors) that is efficient and adaptable in its environment. The development of this robot is also related to the use of advanced sensors to improve its functionality because this robot is widely applied in technological developments and industrial needs. One example in our project, this DDMR robot uses three ultrasonic sensors and the use of IMU sensors. Therefore, the development of this project is very necessary in finding new innovations in its utilization.
## The problem that we have to solve is to provide the features of the DDMR robot that have been provided. These features include the target position of the robot movement by providing a user-defined position point, error detection of the target position that has been determined with the actual conditions, and the utilization of three ultrasonic sensors to detect obstacles.
## The purpose of this project is to gain experience in analyzing a robot that is already available, increase knowledge in programming robotic systems, and increase knowledge in GUI design for monitoring a robot. These things are to improve the abilities possessed by students in the field of mechatronics, especially mechatronics system programming.
## 1\.2 Initial Thought Process
## <a name="_toc149728744"></a>The initial identification that we carried out was to identify every component in the DDMR robot. After that, analyze the schematic series of these components and then think about the concept of providing features to the robot that will be carried out. The initial idea of ​​each stage of the program on the DDMR robot is to move the robot with a target position value that has been determined in the program. After that, try to replace the target position value in the form of a value entered by the user. Followed by conducting trials to detect the error value of a target position value with its actual position. After that, access the ultrasonic sensors which are used to detect obstacles to the robot's journey in reaching its target position. A GUI design will also be prepared to access the robot by providing a target position in the Cartesian plane (x and y axes) in millimeters.
## The challenge in this project is to unite all the features in the above stages in one program and link them to the GUI design. This is a challenge in itself because we don't have experience in this matter. The short time involved in working on this project also meant that we had to be able to work effectively and efficiently.
## The existence of a platform that provides tutorials on this matter can be an opportunity for us to solve this challenge. Guidance from lecturers is also an opportunity for us to complete this DDMR robot project. Support from friends also helps us to learn and process together in working on our respective projects.
## 2 Requirement Analysis and Specification
## <a name="_toc149728745"></a>2.1 User Requirements
## <a name="_toc149728746"></a>The DDMR robot will be operated by students as a learning module in the mechatronics system programming course. The GUI created will display information on data in real time. The data information displayed are robot position, user-defined target position, robot position error value, and ultrasonic sensors. The data displayed can be used by users to monitor and provide position targets for DDMR robots in millimeters.
## 2\.2 System Requirements
`	`In the GUI design, there will be a cartesian coordinate (x and y axis) in millimeters that displays the initial position of the robot (at point 0). After that, the user can provide a target position point by clicking a position on the cartesian coordinates. When the user provides the target position, the DDMR robot will go to the target position with a motor speed that can also be adjusted by the user. There is monitoring of the ideal and actual position errors of the DDMR robot which will be displayed on the GUI design.

`	`If there are objects that block the robot lane in reaching the target position, the DDMR robot will stop. This can be monitored by displaying a distance data on the GUI design generated from the three ultrasonic sensors.
## <a name="_toc149728747"></a>2.3 Tools and Technologies
## The list of software used, namely:
## a.	Arduino IDE
## b.	QT Designer
## List of hardware available on the DDMR robot, namely:
## a.	Arduino Mega 
## b.	3 HC-SR04 ultrasonic sensors
## c.	2 pieces of DC motor JGB37-520 12V
## d.	Raspberry PI 3 model B V1.2
## e.	MPU6050
## f.	2 pieces of XL4005
## g.	Lipo Battery 11.1V 2200mah
## 2\.4 Target specification
*Tabel 1. Tabel caption.*

|**Feature**|**Description**|**Measurement Metric**|**Target Value**|
| :-: | :-: | :-: | :-: |
|Target position|The robot will reach a user-defined target in the form of a point on the cartesian coordinates in the GUI design.|Cartesian coordinates|Millimeters|
|Position error detection|The GUI design will display a target data of the ideal and actual position of the robot.|Scale|Milimeters|
|Avoiding obstacles|The robot will stop when the ultrasonic sensor detects an object near the robot|Scale|Centimeter|

# <a name="_toc149728748"></a>3 Conceptual Design
## <a name="_toc149728749"></a>3.1 System Architecture
Illustrate the high-level architecture of the system, including the GUI and control logic.
## <a name="_toc149728750"></a>3.2 Interface Design
Sketch the preliminary design of the GUI, focusing on user interaction and experience.
## <a name="_toc149728751"></a>3.3 Control Algorithm Design
Outline the design of control algorithms and data processing workflows.
# <a name="_toc149728752"></a>4 Detailed Design and Development
## <a name="_toc149728753"></a>4.1 Component Design
Delve into the design of individual components, modules, and functionalities.
## <a name="_toc149728754"></a>4.2 Coding and Implementation
Document the coding process, adopted standards, and implementation challenges.
## <a name="_toc149728755"></a>4.3 Integration
Discuss the integration of GUI with the control system, and among different system components.
## <a name="_toc149728756"></a>4.4 Unique Features
Highlight any novel features, optimizations or technologies employed.
# <a name="_toc149728757"></a>5 Testing, Evaluation, and Optimization
## <a name="_toc149728758"></a>5.1 Testing Strategy
<a name="_toc149728759"></a>Describe the testing methodologies, cases, and tools used. Emphasize on how the testing validates the targets specified in Section 2.4.
## 5\.2 Performance Evaluation
<a name="_toc149728760"></a>Evaluate the system performance against the defined requirements and objectives. Include a comparative analysis with the targets specified in Section 2.4, illustrating how well the system meets or exceeds these targets.
## 5\.3 Optimization
Discuss any optimizations made to enhance system performance and user experience.
# <a name="_toc149728761"></a>6 Collaboration and Project Management
## <a name="_toc149728762"></a>6.1 Teamwork Dynamics
Reflect on the collaborative endeavor, roles, and contributions of team members.
## <a name="_toc149728763"></a>6.2 Project Management
Document the project timeline, milestones, and management practices adopted.
# <a name="_toc149728764"></a>7 Conclusion and Reflection
## <a name="_toc149728765"></a>7.1 Project Summary
Summarize the key achievements, learnings, and outcomes.
## <a name="_toc149728766"></a>7.2 Future Work
Propose further enhancements, applications, and research directions.
## <a name="_toc149728767"></a>7.3 Personal and Group Reflections
Reflect on the experience, challenges, and acquired knowledge.
# <a name="_toc149728768"></a>8 Appendices
## <a name="_toc149728769"></a>8.1 Bill of Materials
Detail the parts, costs, and sources.
## <a name="_toc149728770"></a>8.2 Electrical Wiring and System Layout
Provide diagrams, schematics, and layout information.
## <a name="_toc149728771"></a>8.3 Code Repository
Include links to the code repository, version control, and change logs.
## <a name="_toc149728772"></a>8.4 Additional Documentation
Include any other relevant documentation, photos, or supporting materials.
# <a name="_toc149728773"></a>9 References
Cite all references, tools, libraries, and external resources used in the project.
6

