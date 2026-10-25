// Generated from original HMPC settings. Do not edit.
// Settings fingerprint: 5a0d0cb06f5fc1c6509f92cbb3f543ca08995fb15bd58ac92ba12a1c2af519d9
#ifndef HMPC_TMPC_SETTINGS_METADATA_H
#define HMPC_TMPC_SETTINGS_METADATA_H
#include <vector>
#include <string>
namespace hmpc_tmpc_settings
{
constexpr int kN = 10;
constexpr int kNu = 2;
constexpr int kNx = 4;
constexpr double kDt = 0.05;
constexpr double kIntegratorStepsize = 0.05;
constexpr int kIntegratorSteps = 1;
constexpr int kNumberOfObjectiveParameters = 6;
constexpr int kNumberOfWeightParameters = 6;
constexpr int kNumberOfConstraintParameters = 72;
constexpr int kNumberOfAcadosParameters = 78;
constexpr int kNDiscs = 1;
constexpr int kRatio = 10;
inline std::vector<double> defaultWeights()
{
  return {1.0, 20.0, 20.0, 20.0, 1.0, 0.2};
}
inline std::vector<std::string> stageVariableNames()
{
  return {"a", "omega", "x", "y", "psi", "v"};
}
inline std::vector<std::string> objectiveParameterNames()
{
  return {"u_x_ref_0", "u_x_ref_1", "u_x_ref_2", "u_x_ref_3", "u_x_ref_4", "u_x_ref_5"};
}
inline std::vector<std::string> weightParameterNames()
{
  return {"R_a", "R_omega", "Q_x", "Q_y", "Q_psi", "Q_v"};
}
inline std::vector<std::string> constraintParameterNames()
{
  return {"disc0_0_a1", "disc0_0_a2", "disc0_0_b", "disc0_1_a1", "disc0_1_a2", "disc0_1_b", "disc0_2_a1", "disc0_2_a2", "disc0_2_b", "disc0_3_a1", "disc0_3_a2", "disc0_3_b", "disc0_4_a1", "disc0_4_a2", "disc0_4_b", "disc0_5_a1", "disc0_5_a2", "disc0_5_b", "disc0_6_a1", "disc0_6_a2", "disc0_6_b", "disc0_7_a1", "disc0_7_a2", "disc0_7_b", "disc0_8_a1", "disc0_8_a2", "disc0_8_b", "disc0_9_a1", "disc0_9_a2", "disc0_9_b", "disc0_10_a1", "disc0_10_a2", "disc0_10_b", "disc0_11_a1", "disc0_11_a2", "disc0_11_b", "disc0_12_a1", "disc0_12_a2", "disc0_12_b", "disc0_13_a1", "disc0_13_a2", "disc0_13_b", "disc0_14_a1", "disc0_14_a2", "disc0_14_b", "disc0_15_a1", "disc0_15_a2", "disc0_15_b", "disc0_16_a1", "disc0_16_a2", "disc0_16_b", "disc0_17_a1", "disc0_17_a2", "disc0_17_b", "disc0_18_a1", "disc0_18_a2", "disc0_18_b", "disc0_19_a1", "disc0_19_a2", "disc0_19_b", "disc0_20_a1", "disc0_20_a2", "disc0_20_b", "disc0_21_a1", "disc0_21_a2", "disc0_21_b", "disc0_22_a1", "disc0_22_a2", "disc0_22_b", "disc0_23_a1", "disc0_23_a2", "disc0_23_b"};
}
inline std::vector<std::string> allParameterNames()
{
  return {"u_x_ref_0", "u_x_ref_1", "u_x_ref_2", "u_x_ref_3", "u_x_ref_4", "u_x_ref_5", "R_a", "R_omega", "Q_x", "Q_y", "Q_psi", "Q_v", "disc0_0_a1", "disc0_0_a2", "disc0_0_b", "disc0_1_a1", "disc0_1_a2", "disc0_1_b", "disc0_2_a1", "disc0_2_a2", "disc0_2_b", "disc0_3_a1", "disc0_3_a2", "disc0_3_b", "disc0_4_a1", "disc0_4_a2", "disc0_4_b", "disc0_5_a1", "disc0_5_a2", "disc0_5_b", "disc0_6_a1", "disc0_6_a2", "disc0_6_b", "disc0_7_a1", "disc0_7_a2", "disc0_7_b", "disc0_8_a1", "disc0_8_a2", "disc0_8_b", "disc0_9_a1", "disc0_9_a2", "disc0_9_b", "disc0_10_a1", "disc0_10_a2", "disc0_10_b", "disc0_11_a1", "disc0_11_a2", "disc0_11_b", "disc0_12_a1", "disc0_12_a2", "disc0_12_b", "disc0_13_a1", "disc0_13_a2", "disc0_13_b", "disc0_14_a1", "disc0_14_a2", "disc0_14_b", "disc0_15_a1", "disc0_15_a2", "disc0_15_b", "disc0_16_a1", "disc0_16_a2", "disc0_16_b", "disc0_17_a1", "disc0_17_a2", "disc0_17_b", "disc0_18_a1", "disc0_18_a2", "disc0_18_b", "disc0_19_a1", "disc0_19_a2", "disc0_19_b", "disc0_20_a1", "disc0_20_a2", "disc0_20_b", "disc0_21_a1", "disc0_21_a2", "disc0_21_b", "disc0_22_a1", "disc0_22_a2", "disc0_22_b", "disc0_23_a1", "disc0_23_a2", "disc0_23_b"};
}
inline std::vector<std::string> constantNames()
{
  return {"ratio", "use_input_rates", "use_slack", "integrator_stepsize", "integrator_steps", "Q", "R", "reference_dependent_terminal", "X_coef_0", "Y_coef_0", "X_coef_1", "Y_coef_1", "X_coef_2", "Y_coef_2", "X_coef_3", "Y_coef_3", "X_coef_4", "Y_coef_4", "alpha"};
}
inline std::vector<int> constantDimensions()
{
  return {0, 0, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0};
}
inline std::vector<int> constantSizes()
{
  return {1, 1, 1, 1, 1, 4, 4, 2, 2, 1, 4, 4, 2, 4, 4, 4, 2, 4, 4, 4, 2, 4, 4, 4, 2, 4, 4, 4, 2, 4, 1};
}
inline std::vector<int> constantStartIndices()
{
  return {0, 1, 2, 3, 4, 5, 21, 25, 26, 42, 50, 66, 74, 90, 98, 114, 122, 138, 146};
}
inline std::vector<double> constantValues()
{
  return {10.0, 0.0, 0.0, 0.05, 1.0, 20.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 20.0, 1.0, 0.000141151004565431, -7.64977549497822e-15, 5.317292572934272e-13, -2.231348183898043e-13, -7.64977549497822e-15, 0.0001411510045530988, 1.0527180333705664e-12, 1.3628687271940129e-13, 5.317292572934272e-13, 1.0527180333705664e-12, 0.0006735710255713753, 8.524831017104557e-12, -2.231348183898043e-13, 1.3628687271940129e-13, 8.524831017104557e-12, 0.010243238129164414, -2.5511953855717846e-08, -3.126971198954128e-08, -5.775787295585614e-09, -3.315655874917329e-08, 1.2950679236959966e-05, -0.04224120985103885, -0.24011351681355536, 1.2946415878894098e-05, -7.15210932788394e-15, 2.5607753158795083e-15, -4.589909779370446e-14, -7.615359506686891e-05, 2.5607753158795083e-15, 1.058411037476143e-15, -6.155455453635649e-16, 5.999575842382982e-17, -4.589909779370446e-14, -6.155455453635649e-16, -7.867584427957155e-15, -1.0564855063990394e-12, -7.615359506686891e-05, 5.999575842382982e-17, -1.0564855063990394e-12, 4.1942830152515665e-13, -0.011358773981499898, -1.9352676216802486e-07, -8.566688614435352e-12, -6.370753271517066e-14, 2.538502954406894e-09, 6.03717767155106e-11, 1.684531898464183e-06, 2.0913997590229458e-06, -1.0465630718862481e-15, -4.123764620089515e-15, -7.700824447134739e-17, 1.1453966744211745e-15, -4.123764620089515e-15, 4.38635690905521e-15, -4.5519583528061784e-14, -7.615359506077796e-05, -7.700824447134739e-17, -4.5519583528061784e-14, 7.0031357894915314e-15, -1.473635480605989e-12, 1.1453966744211745e-15, -7.615359506077796e-05, -1.473635480605989e-12, -1.9127481292236335e-12, 7.452343726781355e-12, -1.8497921100857193e-13, -0.011358774007948517, -1.935260398933171e-07, 3.5209965618163113e-09, -1.7456375075794908e-10, 4.403890208893343e-07, 2.2175477577233317e-06, -5.330322545575999e-14, -5.1912689237517466e-14, 5.499012196688625e-05, 7.001332352877292e-13, -5.1912689237517466e-14, -1.0178165450286272e-15, -7.36150344033668e-17, -3.580182674449627e-14, 5.499012196688625e-05, -7.36150344033668e-17, 1.5313865426905971e-12, -1.5362544015493073e-13, 7.001332352877292e-13, -3.580182674449627e-14, -1.5362544015493073e-13, 7.396282350243773e-15, 4.5832416199249063e-07, 0.00027460468284768976, -7.812747115456353e-10, 5.914180631235363e-11, -7.208727480793693e-05, -8.835097565989743e-05, 8.105654074891835e-08, 1.3063404315188463e-08, 1.6131469582750982e-15, 2.6207422024866713e-14, 6.706153928763552e-16, 3.653101174392477e-14, 2.6207422024866713e-14, 1.0569992324225421e-13, -5.49901219630555e-05, -6.811411177233914e-13, 6.706153928763552e-16, -5.49901219630555e-05, -2.141433470427119e-12, -6.926133275474842e-13, 3.653101174392477e-14, -6.811411177233914e-13, -6.926133275474842e-13, -1.070579274345546e-14, -1.0761892160113423e-09, 5.591382365131748e-11, -4.5841906861278504e-07, -0.00027460468193961796, 1.6320607604896348e-05, 9.368261769432333e-05, -2.3017132296205354e-07, 4.4721235328336e-09, 0.38187097195048747};
}
}
#endif  // HMPC_TMPC_SETTINGS_METADATA_H
