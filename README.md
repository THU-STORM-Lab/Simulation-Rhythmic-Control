-----

# Rhythmic Control of Automated Traffic: Simulation Repository

This repository contains the official MATLAB simulation code for the research paper:

> Chen, X., Li, M., Lin, X., Yin, Y., & He, F. (2021). **Rhythmic Control of Automated Traffic-Part I: Concept and Properties at Isolated Intersections**. *Transportation Science*, 55(5), 969-987. [https://doi.org/10.1287/trsc.2021.1060](https://doi.org/10.1287/trsc.2021.1060)

The code allows users to reproduce the simulation results presented in the paper and to experiment with the different intersection control strategies discussed.

-----

## File Descriptions

This repository includes the following files:

  * **`src/Simulation_RC.m`**: A MATLAB function that simulates the **Rhythmic Control (RC)** scheme. [cite\_start]This is the core concept of the paper, where connected and automated vehicles (CAVs) are assigned rhythmic, conflict-free entry times to pass through an intersection without stopping[cite: 18, 62].

  * **`src/Simulation_TSC.m`**: A MATLAB function that simulates a traditional **Traffic Signal Control (TSC)** system, adapted for a CAV environment. This model uses the classic Webster's method to optimize signal timings and serves as a primary benchmark for performance comparison.

  * **`src/Simulation_RCTS.m`**: A MATLAB function that simulates a hybrid **Rhythmic Control - Traffic Signal (RC-TS)** scheme. This model is designed to improve performance in highly imbalanced traffic demand scenarios by combining the benefits of both RC and dedicated signal phases.

  * **`src/input.mat`**: A MATLAB data file containing the input parameters required to run the simulations. This includes the traffic demand vector (`d`), vehicle dimensions (`L`, `W`), and other physical constants that align with the "highly imbalanced demand scenario" (`d_h`) described in the paper.

-----

## System Requirements

  * **MATLAB**: All simulation scripts are written in MATLAB.
  * **Statistics and Machine Learning Toolbox**: The code uses the `exprnd` function to generate random vehicle arrivals. This function is part of this toolbox.

-----

## How to Cite

If you use this code in your research, please cite the original paper:

```bibtex
@article{chen2021rhythmic,
  title={Rhythmic control of automated traffic—Part I: Concept and properties at isolated intersections},
  author={Chen, Xiangdong and Li, Meng and Lin, Xi and Yin, Yafeng and He, Fang},
  journal={Transportation Science},
  volume={55},
  number={5},
  pages={969--987},
  year={2021},
  publisher={INFORMS}
}
```
