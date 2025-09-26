# ILDMPC
It's the iterative learning distributed model predictive control (ILDMPC) for 2-dimensional autonomous vehicle platoons (AVPs). 
## citation
If they are helpful in your research, please cite the following information:
```bibtex
@ARTICLE{11130514,
  author={Zhang, Nianhua and Chen, Jicheng and Viadero-Monasterio, Fernando and Zhang, Hui},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={Iterative Learning Distributed Model Predictive Control for Autonomous Vehicle Platoons With Applications to Repetitive Tasks}, 
  year={2025},
  volume={},
  number={},
  pages={1-15},
  doi={10.1109/TITS.2025.3597322}}

```
## introduction
AVPs are important components of intelligent transportation systems and are quite powerful for repetitive tasks, where the guiding vehicle and following vehicles require different control strategies with different objectives and the there are various 
conditions of different tasks and different combinations of vehicles. 
To address the control problem, we combined the iterative learning control (ILC) and the model predictive control (MPC) to design an ILDMPC controller for AVPs considering the combined lateral and longitudinal vehicle dynamics.
Simulations are carried out to demonstrate the effectiveness, where the proposed controller is validated to evolve existing control laws and provide almost 30\% improvement quantified with the error-based indicator. 
## result
There are two videos of a 4-vehicle AVP, where the one is controlled by a compared feedback controller, and the other is controlled by the proposed ILDMPC controller. 
The first one is the result with the compared feedback controller, where a dangerous collision occurs. 
![platoon_compared](https://github.com/ZNianHua/ILDMPC/blob/main/ILDMPC_video_iteration0.mat.gif)

The second one is the result with the ILDMPC controller, where performances are improved. 
![platoon_ILCMPC](https://github.com/ZNianHua/ILDMPC/blob/main/ILDMPC_video_iteration14.mat%2000_00_00-00_00_30.gif)
## contact
e-mail: znh943161859@163.com

