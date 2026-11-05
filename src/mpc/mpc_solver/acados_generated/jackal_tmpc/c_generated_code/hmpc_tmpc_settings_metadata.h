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
  return {10.0, 0.0, 0.0, 0.05, 1.0, 20.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.0, 20.0, 1.0, 0.00023136113285716429, 2.306409760890037e-13, -1.9166520502722068e-13, -1.7114096596407715e-12, 2.306409760890037e-13, 0.0002313611326784095, 8.634833083041929e-13, 1.586989329954232e-12, -1.9166520502722068e-13, 8.634833083041929e-13, 0.0013354704433546707, -2.0996567183240113e-11, -1.7114096596407715e-12, 1.586989329954232e-12, -2.0996567183240113e-11, 0.018142398057103767, 1.0086996544203291e-10, 1.8941066068639947e-13, -8.953239267419433e-11, 2.7120149442577702e-11, -4.272831071028089e-09, -0.02915766858528705, -0.47925779563599036, -1.0771356116630083e-09, 2.2130462852308551e-13, -7.422880940384571e-14, -6.114349954264197e-14, -0.0001710345286453441, -7.422880940384571e-14, -7.774380675013305e-14, 5.021492064889923e-14, 8.03422327233972e-14, -6.114349954264197e-14, 5.021492064889923e-14, 6.90098373000931e-13, 4.3529745110911154e-11, -0.0001710345286453441, 8.03422327233972e-14, 4.3529745110911154e-11, 1.0448662537454943e-10, 0.0033224088108638333, 4.4204488297862297e-11, -2.1691821615752855e-12, 3.625381259025141e-13, -1.7854899927176427e-09, -9.276036208247151e-11, -4.537153267816073e-09, -6.725408611296722e-10, -4.054587633673545e-14, 1.4442132416154109e-13, 1.911032627764946e-14, 1.8615642717047204e-13, 1.4442132416154109e-13, -1.9135407874015688e-13, -1.2609141119948432e-13, -0.00017103452863611102, 1.911032627764946e-14, -1.2609141119948432e-13, 3.5643119580435244e-13, 5.202713951831192e-11, 1.8615642717047204e-13, -0.00017103452863611102, 5.202713951831192e-11, -1.952004775812557e-10, -6.581802514696844e-12, -3.3886287713315376e-14, 0.003322408808267207, 4.359942195337533e-11, -2.2328698438783485e-09, 1.3386195571539265e-10, 1.1166617144555596e-08, -1.0454696608922196e-09, 3.817807945601132e-14, -4.4473665549665324e-14, 9.67649790249642e-05, -1.0038767563738672e-12, -4.4473665549665324e-14, -1.251639390307592e-13, -2.876881168503206e-14, -1.2748676405358266e-13, 9.67649790249642e-05, -2.876881168503206e-14, -4.814222613197231e-11, -2.928296611574253e-11, -1.0038767563738672e-12, -1.2748676405358266e-13, -2.928296611574253e-11, 1.556468079400102e-12, -3.651692753551122e-10, 0.00023361173661913567, 1.7226170563620795e-11, -2.89614823693021e-13, -1.0153909716527577e-08, 1.9200744190532345e-09, 8.38980635294407e-10, -1.1460089546843827e-09, 1.6450918540325642e-13, -7.751605017635201e-14, 7.710842962886331e-14, -1.9627765067789363e-13, -7.751605017635201e-14, 2.4831902097739684e-13, -9.67649789984813e-05, 1.045299905698742e-12, 7.710842962886331e-14, -9.67649789984813e-05, 5.736971410917574e-11, -5.537274092045318e-11, -1.9627765067789363e-13, 1.045299905698742e-12, -5.537274092045318e-11, -2.071612294290945e-12, 2.3590843847548208e-11, 6.582824409345122e-13, 3.5767716682724485e-10, -0.00023361173768358473, -8.046689638917034e-09, -2.5760581116669624e-09, -1.2619369663851285e-09, -5.423185571631799e-10, 0.2809774325855172};
}
}
#endif  // HMPC_TMPC_SETTINGS_METADATA_H
