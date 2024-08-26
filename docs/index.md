# Johnson Loh

## Biography

My professional passion is to improve general human health and well-being through technological solutions.
As a generalist with many interests, I like to challenge myself in new application fields with a recent focus on bio-inspired computing in biomedical signal monitoring and I am looking forward to my next career challenge.
I thrive most in multi-disciplinary projects and I like to solve them in a multi-cultural harmonious team. 
I strongly believe that the creativity and high ethical standards are best fostered in a diverse team and are necessary to solve the challenges of the future.

??? note "Project Summary PhD"
    - PhD Candidate at [Chair of Integrated Digital Systems and Circuit Design, RWTH Aachen University](https://www.ids.rwth-aachen.de/en/)
    - Project coordination of [BMBF Clusters4Future](https://www.bmbf.de/bmbf/de/forschung/zukunftsstrategie/zukunftscluster-initiative-clusters4future/zukunftscluster-initiative-clusters4future_node.html), [NeuroSys Project C, Algorithm-Hardware Co-Design](https://neurosys.info/projekte/)
    - DNN architecture design and optimization
        - Reduced complexity domain generalization algorithms for co-optimized ANNs ([Paper](https://dx.doi.org/10.1109/TBCAS.2024.3418085))
        - Lossless sparse temporal coding for ANN-SNN conversion ([Paper](https://dx.doi.org/10.23919/DATE56975.2023.10137112))
        - Dataflow optimization for subsampling-based TCN accelerators (incl. tapeout in GF22FDX; [Paper1](https://dx.doi.org/10.1109/norcas57515.2022.9934591), [Paper2](https://dx.doi.org/10.1109/tvlsi.2023.3329360))
        - High quality atrial fibrillation classification using downsized convolutional neural network ([Paper](https://dx.doi.org/10.1109/ASAP49362.2020.00042))
        - Energy estimation of ANN inference for computer vision models with different dataflows using simplified memory model
    - General ML explorations and profiling
        - Exploration of ML architectures (e.g. Autoencoder, LSTM, GRU) and methods (e.g. data augmentation, quantization, partial NN update etc.) for ECG classification/anomaly detection
        - [BMBF Innovationswettbewerb "Elektronik für energiesparsame Informations- und Kommunikationstechnik"](https://www.elektronikforschung.de/service/aktuelles/innovationswettbewerb-elektronik-fuer-energiesparsame-informations-und-kommunikationstechnik) - GIGA, Traffic monitoring using acoustic MEMS sensors and beamforming
        - Hierarchical cascaded classifier exploration ([Paper](https://dx.doi.org/10.1007/978-3-030-95470-3_32), [Patent](https://worldwide.espacenet.com/patent/search/family/077175963/publication/DE102020202391A1?q=DE102020202391%20A1))
    - Contributions to other projects
        - Vehicle surroundings perception ([Paper](https://dx.doi.org/10.1002/aisy.202300679))
        - Modified split learning ([Paper](https://dx.doi.org/10.1109/icics60529.2023.10330472))
        - Depthwise separable convolution accelerator for computer vision models ([Paper](https://dx.doi.org/10.1109/vlsi-soc57769.2023.10321918))
        - [Intel neuromorphic DNS challenge](https://github.com/IntelLabs/IntelNeuromorphicDNSChallenge) - Submission: [Noice](https://github.com/thebarnable/noice)
        - Structured data path design ([Paper](https://dx.doi.org/10.1109/islped58423.2023.10244608))
        - Finite impulse response (FIR) filter exploration study in TSMC28 for high throughput digital echo cancellation (DEC)

??? note "Project Summary Master"
    - Master studies at RWTH Aachen University in Electrical Engineering, Information Technology and Computer Engineering 
    - Simultaneous Localization and Mapping (SLAM) and basic navigation for [SeekurJr robot](https://robots.ros.org/seekurjr/) in [VEROSIM](https://www.verosim-solutions.com/) (Master's Thesis at [Institute for Man-Machine Interaction](https://www.mmi.rwth-aachen.de/en/)) ([Thesis (ger)](./assets/files/ma_thesis.pdf))
        - Seekur Jr is equipped with 3 front, 1 rear camera (calibrated/interfaced with OpenCV) and one front LIDAR
        - Extended and Unscented Kalman Filter with extensible state variable for SLAM
        - SLAM map consists of hybrid grid and feature map in 2D space
        - Navigation is realized with global gridmap-based path planning and local collision avoidance
        - Implementation is integrated into proprietary VEROSIM environment (custom micro kernel architecture) with numerical computing libraries, e.g. Eigen library
    - Integration of SLAM features for existing ROS framework (Internship at [Fraunhofer IPA group](https://github.com/ipa320), now: [NODE Robotics](https://node-robotics.com/)) ([Report (ger)](./assets/files/intern_report.pdf))
        - Implementation of adaptive partical filter with KLD sampling
        - Implementation of GPS and RFID sensor modules with modular integration into ROS framework
            - Sensor fusion through particle filter algorithm in SLAM implementation
        - Implementation of map conversion algorithms between low-resolution normal distribution transform (NDT) and high-resolution gridmaps
            - Conversion algorithm is partially based on classifical computer vision methods, e.g. Bresenham adapted to ellipsoids or wavefront algorithms 
    - Implementation of SLAM and navigation features in [PenguiPi robot](https://github.com/qcr/PenguinPi-robot) (Exchange semester in Queensland University of Technology)
        - Beacon-based localization detected through commercially available camera modules
        - Image pre-processing and noise reduction for reliable beacon detection in distance
        - Motor encoder and actuation calibration for reliable navigation in unknown environment
        - [(Backup Repo)](https://github.com/bobinator7/EGB439)
    - R&D support for cardiac support systems (Student research assistant at [Institute of Automatic Control](https://www.irt.rwth-aachen.de/go/id/iung/?lidx=1))
        - Prototype development of printed circuit board (PCB) designs for sensor read-out and actuation
        - Embedded programming of microcontrollers in C/C++ and basic system modeling using standardized open-source specifications like functional mock-up interface (FMI)

??? note "Project Summary Bachelor"
    - Bachelor studies at RWTH Aachen University in Electrical Engineering, Information Technology and Computer Engineering
        - Analysis and Control of a Mechanical-Rotational Impedance Actuator - [MeRIA](https://publications.rwth-aachen.de/record/780722/files/780722.pdf) (Bachelor's Thesis at [Medical Information Technology (MedIT), Helmholtz-Institute for Biomedical Engineering](https://www.medit.hia.rwth-aachen.de/en/)) ([Thesis (ger)](./assets/files/ba_thesis.pdf))
            - Modeling of a variable stiffness actuator based on a cantilever design with variable beam length
            - Cascaded control of torque and position using PID controllers
            - Implementation of safety mechanisms for emergency power shutdowns
            - Implementations are performed in a MATLAB/Simulink environment
                - Control unit and sensors are interfaced with a [dSpace module](https://www.dspace.com/de/gmb/home/products/hw/singbord/conledpanels.cfm) in a hardware-in-the-loop setup
        - Support of teaching activities for ["Introduction to Medical Engineering"](https://moodle.rwth-aachen.de/enrol/index.php?id=33738), ["Mathematical System Theory"](https://moodle.rwth-aachen.de/enrol/index.php?id=22706) and ["Fundamentals of Electromagnetic Field Theory"](https://moodle.rwth-aachen.de/enrol/index.php?id=32215)
            - Process automization of script generation for lecture supporting exercises
            - Supervision and support of student groups up to 30

## Skills

### Technical Skills
- (Bio-inspired) Neural network design and optimization (Pytorch, Tensorflow)
- Real-time digital signal processing (incl. hardware accelerators for DNNs)
- ASIC/FPGA design flow from circuit-level to RTL-level design (Cadence, Siemens/Mentor, Synopsys EDA tools; Xilinx/Altera)
- Data analysis (Scipy, MATLAB etc.)
- Programming and Scripting (C/C++, MATLAB/Simulink, Python, Perl, Tcl etc.)
- Robotics (Sensor-fusion, i.e. Camera, LIDAR etc.; SLAM)
- Scientific writing and presentation (LaTex & MS Office)

### Soft Skills
- Experience in leadership and decision-making
- Project management, coordination and execution
- Science communication for broad and specialized audience
- Initiation of workflows and organizational tooling for more efficient collaboration (e.g. research data management, code version control etc.)
- Inter-cultural and multi-disciplinary teamwork

## Disseminations

### Publications ([Google Scholar](https://scholar.google.de/citations?user=_vybVlsAAAAJ))

- **J. Loh**, L. Dudchenko, J. Viga and T. Gemmeke, "Towards Hardware Supported Domain Generalization in DNN-based Edge Computing Devices for Health Monitoring," IEEE Transactions on Biomedical Circuits and Systems, Jun. 2024, doi: [10.1109/TBCAS.2024.3418085](https://dx.doi.org/10.1109/TBCAS.2024.3418085).

- M. Lahr, **J. Loh**, M. Schwarz, M.Lemme, T. Gemmeke, "Vehicle Surroundings Perception using MEMS Inertial Sensors," Advanced Intelligent Systems, Apr. 2024, doi: [10.1002/aisy.202300679](https://dx.doi.org/10.1002/aisy.202300679).

- A. Ayad, M. Barhoush, **J. Loh**, T. Gemmeke, and A. Schmeink, "PEACE: Private and energy-efficient algorithm for cardiac evaluation on the edge using modified split learning and model quantization," in 2023 14th International Conference on Information and Communication Systems (ICICS), IEEE, Nov. 2023. doi: [10.1109/icics60529.2023.10330472](https://dx.doi.org/10.1109/icics60529.2023.10330472).

- **J. Loh** and T. Gemmeke, "Stream processing architectures for continuous ecg monitoring using subsampling-based classifiers," IEEE Transactions on Very Large Scale Integration (VLSI) Systems, pp. 1–11, 2023, ISSN: 1557-9999. doi: [10.1109/tvlsi.2023.3329360](https://dx.doi.org/10.1109/tvlsi.2023.3329360).

- Y. Chen, J. Lou, C. Lanius, F. Freye, **J. Loh**, and T. Gemmeke, "An energy-efficient and area-efficient depthwise separable convolution accelerator with minimal on-chip memory access," in 2023 IFIP/IEEE 31st International Conference on Very Large Scale Integration (VLSI-SoC), IEEE, Oct. 2023. DOI: [10.1109/vlsi-soc57769.2023.10321918](https://dx.doi.org/10.1109/vlsi-soc57769.2023.10321918).

- C. Lanius, J. Lou, **J. Loh**, and T. Gemmeke, "Automatic generation of structured macros using standard cells – application to CIM," in 2023 IEEE/ACM International Symposium on Low Power Electronics and Design (ISLPED), IEEE, Aug. 2023. DOI: [10.1109/islped58423.2023.10244608](https://dx.doi.org/10.1109/islped58423.2023.10244608).

- **J. Loh** and T. Gemmeke, "Lossless Sparse Temporal Coding for SNN-based Classification of Time-Continuous Signals," 2023 Design, Automation & Test in Europe Conference & Exhibition (DATE), Antwerp, Belgium, 2023, pp. 1-6, doi: [10.23919/DATE56975.2023.10137112](https://dx.doi.org/10.23919/DATE56975.2023.10137112).

- **J. Loh** and T. Gemmeke, "Dataflow optimizations in a sub-uW data-driven TCN accelerator for continuous ECG monitoring," in 2022 IEEE Nordic Circuits and Systems Conference (NorCAS), IEEE, Oct. 2022. DOI: [10.1109/norcas57515.2022.9934591](https://dx.doi.org/10.1109/norcas57515.2022.9934591).

- C. Latotzke, **J. Loh**, and T. Gemmeke, "Cascaded classifier for pareto-optimal accuracy-cost trade-off using off-the-shelf ANNs," in Machine Learning, Optimization, and Data Science, Springer International Publishing, 2022, pp. 423–435. DOI: [10.1007/978-3-030-95470-3\_32](https://dx.doi.org/10.1007/978-3-030-95470-3_32).

- **J. Loh**, J. Wen and T. Gemmeke, "Low-Cost DNN Hardware Accelerator for Wearable, High-Quality Cardiac Arrythmia Detection," 2020 IEEE 31st International Conference on Application-specific Systems, Architectures and Processors (ASAP), Manchester, UK, 2020, pp. 213-216, doi: [10.1109/ASAP49362.2020.00042](https://dx.doi.org/10.1109/ASAP49362.2020.00042).

### Patent Applications

- M. Lahr, R. Foell, T. Gemmeke and **J. Loh**, "PKW-Umgebungswahrnehmung mit MEMS Inertialsensoren und digitaler Auswertelogik," German Patent and Trade Mark Office, *submitted Aug. 4, 2023*.

- T. Gemmeke, **J. Loh**, C. Höffler, A. Schmeink and A. Ayad, "Energieoptimierte Vorrichtung zur Klassifizierung von Daten," German Patent and Trade Mark Office, Patent No. [DE102020202391A1](https://worldwide.espacenet.com/patent/search/family/077175963/publication/DE102020202391A1?q=DE102020202391%20A1), *published Aug. 26, 2021*, (discontinued Jan. 15, 2022).

## Teaching

- **Supervised Students**
    - Jianan Wen (M.Sc. - Thesis), 2020 - Logic design of an Energy-Efficient CNN for ECG Classification based on Wavelet Transformation
    - Qiwei Zhang (M.Sc. - Thesis), 2020 - Automated Design Space Exploration for Hardware Implementations of Structured Datapaths
    - Junhong Li (M.Sc. - Thesis), 2020 - Exploration of Hardware Architectures for ECG Classification using ResNet Architectures
    - Louis Cherel (M.Sc. - Thesis), 2021 - Hardware Architectures for 1D Sparsely Encoded Neural Networks
    - Maxim Drobjazko (M.Sc. - Thesis), 2022 - Recurrent NN Hardware-Architectures for Time-Continuous Biomedical Signals
    - Justus Viga (M.Sc. - Thesis), 2022 - In-Field Individualisation of Classification-/Detectionsystems for Biomedical Signals
    - Adeola Adebayo (M.Sc. - Thesis), 2023 - Hardware-Supported On-the-fly Domain Generalization for ECG Classification
    - Additional 6 student research assistants (+ 4 co-supervised) 
  <!-- - Student research assistants: Toma Gavric, Yu-Hsuan Tai, Abdulrahman Ali Hasan, Lyubov Dudchenko, Vinh Hai Luong, Niloufar Bateni, Tim Stadtmann (co-supervised), Amin Abbasloo (co-supervised), Zain Methab (co-supervised), Nivin Clinton Arokyaraj (co-supervised) -->
- **Lectures and Labs**
    - Supervision of 
        - Seminar on Selected Topics of Integrated Digital Systems - "Neuromorphic Architectures - A Feasibility Study for Real-Time Edge Computing", RWTH Aachen University (M.Sc.)
        - Computer Arithmetic - Fundamentals, RWTH Aachen University (M.Sc.)
        - VLSI Design for Digital Signal Processing - Architectures, RWTH Aachen University (M.Sc.)
        - VLSI Design Lab, RWTH Aachen University (M.Sc.)
        - FPGA Design Lab, RWTH Aachen University (M.Sc.)
        - Micro-/Nanoelectronics Lab - VLSI and FPGA Design Fundamentals, RWTH Aachen University (B.Sc.)
    - Additional support for 
        - Computer Arithmetic - Advanced, RWTH Aachen University (M.Sc.)
        - Artificial Neural Networks, RWTH Aachen University (M.Sc.)
    - [(Website)](https://www.ids.rwth-aachen.de/en/teaching)

## Services
- Reviewer for [IEEE Transactions on Biomedical Circuits and Systems (TBioCAS)](https://ieee-cas.org/publication/TBioCAS)
- Reviewer for [International Conference on Machine Learning, Optimization, and Data Science (LOD)](https://lod2023.icas.cc/)

## Miscellaneous

### Languages
- German: Mother tongue (spoken & written)
- English: Fluent (spoken & written)
- Cantonese: Mother tongue (spoken)
- Mandarin: Basic (spoken & written)

### Extra-curricular Activities
- Co-founder - neuroAIx e.V., non-profit organization for the support of science, research and education for activities at the Chair of Integrated Digital Systems and Circuit Design ([Website](https://neuroaix.de/))
- Co-founder - Gelofanga UG, organization for consulting and services in the field of micro- and nano-electronics 
- Mentor - [International Academy, RWTH Aachen University](https://www.academy.rwth-aachen.de/en/programs/short-courses/mentoring-program)
- Mentor - [International Office BeBuddy Program, RWTH Aachen University](https://www.rwth-aachen.de/cms/root/studium/im-studium/engagement-freizeit/engagement-international/~bpei/bebuddy/?lidx=1)