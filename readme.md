[SVM_ENMPC_Low_Disc_Code]

***************************************************************************

Copyright (c) 2016, Ankush Chakrabarty
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
***************************************************************************

***************************************************************************
REFERENCE
***************************************************************************

If this code is helpful to you, please cite:

[BiBTeX]
@article{chakrabarty2017support,
  title={Support vector machine informed explicit nonlinear model predictive control using low-discrepancy sequences},
  author={Chakrabarty, Ankush and Dinh, Vu and Corless, Martin J and Rundell, Ann E and {\.Z}ak, Stanis{\l}aw H and Buzzard, Gregery T},
  journal={IEEE Transactions on Automatic Control},
  volume={62},
  number={1},
  pages={135--148},
  year={2017},
  publisher={IEEE}
}

[1] Chakrabarty, A., Dinh, V., Corless, M.J., Rundell, A.E., ?ak, S.H. and Buzzard, G.T., 2017. 
Support vector machine informed explicit nonlinear model predictive control using low-discrepancy sequences. 
IEEE Transactions on Automatic Control, 62(1), pp.135-148.

***************************************************************************
README
***************************************************************************

To solve the nonlinear MPC problem, we use a barrier function method to enforce
state constraints and the GODLIKE solver (https://github.com/rodyo/FEX-GODLIKE).

File Descriptions (in order of full algorithm, see [1]):

1) main_collect_samples:        generates low-discrepancy Halton samples and solves the
                                NMPC problem at each sample to collect feasibility and
                                control information.

2) main_feasible_region_SVM:    generates an approximation of the feasible region using
                                samples collected in main_collect_samples.m.
                                The inner approximation is not automated in this version
                                but can be constructed by taking sublevel sets of the SVM
                                decision boundary enforced by the variable 'feas_threshold'.
                                For complete automation of the inner approximation, we
                                recommend using scikit-learn in Python.
                                After constructing the inner approximation, resampling is
                                performed to get a rich enough dataset for controller construction.

3) main_approx_NMPC:            constructs approximate NMPC using Chebyshev polynomials and
                                regression and tests the controller in closed-loop using the 
                                Simulink file 'simulation_ENMPC.slx' in continuous-time.

Function list:

1) constraint_violated:         checks state constraint violation

2) find_optimal_NMPC:           computes optimal NMPC action given a state and constraints

3) model:                       implements nonlinear model investigated in Chen and Allgower,
                                Automatica, 1998.

4) objFncMPC:                   MPC objective function (quadratic cost can be altered to other cost)