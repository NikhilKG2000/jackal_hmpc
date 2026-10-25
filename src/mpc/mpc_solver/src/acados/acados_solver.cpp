#include "acados/acados_solver.h"
#include "hmpc_tmpc_settings_metadata.h"
#include "hmpc_pmpc_settings_metadata.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigen>

#include <acados/utils/types.h>

#include <mpc_solver/core/helper_functions.h>

namespace
{

constexpr int kN = hmpc_tmpc_settings::kN;
constexpr int kNu = hmpc_tmpc_settings::kNu;
constexpr int kNx = hmpc_tmpc_settings::kNx;

static_assert(kN == JACKAL_TMPC_N,
              "TMPC horizon metadata differs from solver");
static_assert(kNu == JACKAL_TMPC_NU,
              "TMPC input dimensions differ from solver");
static_assert(kNx == JACKAL_TMPC_NX,
              "TMPC state dimensions differ from solver");
constexpr int kNumberOfObjectiveParameters =
  hmpc_tmpc_settings::kNumberOfObjectiveParameters;
constexpr int kNumberOfWeightParameters =
  hmpc_tmpc_settings::kNumberOfWeightParameters;
constexpr int kNumberOfConstraintParameters =
  hmpc_tmpc_settings::kNumberOfConstraintParameters;
constexpr int kNumberOfAcadosParameters =
  hmpc_tmpc_settings::kNumberOfAcadosParameters;


namespace pmpc_data
{

constexpr int kN = hmpc_pmpc_settings::kN;
constexpr int kNu = hmpc_pmpc_settings::kNu;
constexpr int kNx = hmpc_pmpc_settings::kNx;

static_assert(kN == JACKAL_PMPC_N,
              "PMPC horizon metadata differs from solver");
static_assert(kNu == JACKAL_PMPC_NU,
              "PMPC input dimensions differ from solver");
static_assert(kNx == JACKAL_PMPC_NX,
              "PMPC state dimensions differ from solver");
constexpr int kNumberOfObjectiveParameters =
  hmpc_pmpc_settings::kNumberOfObjectiveParameters;
constexpr int kNumberOfWeightParameters =
  hmpc_pmpc_settings::kNumberOfWeightParameters;
constexpr int kNumberOfConstraintParameters =
  hmpc_pmpc_settings::kNumberOfConstraintParameters;
constexpr int kNumberOfAcadosParameters =
  hmpc_pmpc_settings::kNumberOfAcadosParameters;


}  // namespace pmpc_data

}  // namespace

namespace mpc_solver
{

JackalTmpcAcadosSolver::JackalTmpcAcadosSolver()
  : SolverBase(
      hmpc_tmpc_settings::kDt,
      kN,
      kNx,
      kNu,
      kNx,
      hmpc_tmpc_settings::stageVariableNames(),
      hmpc_tmpc_settings::constantNames(),
      hmpc_tmpc_settings::constantDimensions(),
      hmpc_tmpc_settings::constantSizes(),
      hmpc_tmpc_settings::constantStartIndices(),
      hmpc_tmpc_settings::constantValues(),
      kNumberOfObjectiveParameters + kNumberOfWeightParameters +
        kNumberOfConstraintParameters,
      hmpc_tmpc_settings::allParameterNames(),
      kNumberOfObjectiveParameters,
      hmpc_tmpc_settings::objectiveParameterNames(),
      kNumberOfWeightParameters,
      hmpc_tmpc_settings::weightParameterNames(),
      hmpc_tmpc_settings::kNDiscs,
      kNumberOfConstraintParameters,
      hmpc_tmpc_settings::constraintParameterNames(),
      {"ReferenceTrajectory", "StaticPolyhedronConstraints"})
{
  capsule_ = jackal_tmpc_acados_create_capsule();
  if (capsule_ == nullptr)
  {
    throw std::runtime_error("Could not allocate Jackal TMPC acados capsule");
  }

  const int create_status = jackal_tmpc_acados_create(capsule_);
  if (create_status != 0)
  {
    jackal_tmpc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    throw std::runtime_error(
      "Could not create Jackal TMPC acados solver, status " +
      std::to_string(create_status));
  }

  nlp_config_ = jackal_tmpc_acados_get_nlp_config(capsule_);
  nlp_dims_ = jackal_tmpc_acados_get_nlp_dims(capsule_);
  nlp_in_ = jackal_tmpc_acados_get_nlp_in(capsule_);
  nlp_out_ = jackal_tmpc_acados_get_nlp_out(capsule_);
  nlp_solver_ = jackal_tmpc_acados_get_nlp_solver(capsule_);

  initializeDefaultGuess();
  loadAllParameters();
  loadStatesMatrixInSolver();
}

JackalTmpcAcadosSolver::~JackalTmpcAcadosSolver()
{
  if (capsule_ != nullptr)
  {
    jackal_tmpc_acados_free(capsule_);
    jackal_tmpc_acados_free_capsule(capsule_);
  }
}

int JackalTmpcAcadosSolver::constantIndexOrThrow(
  const std::string& constant_name) const
{
  const int index = returnDataPosition(constant_name, constants_names_);
  if (index < 0)
  {
    throw std::out_of_range("Unknown solver constant: " + constant_name);
  }
  return index;
}

bool JackalTmpcAcadosSolver::getConstantScalarBool(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]] != 0.0;
}

int JackalTmpcAcadosSolver::getConstantScalarInt(std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return static_cast<int>(constants_[constants_start_idc_[index]]);
}

double JackalTmpcAcadosSolver::getConstantScalarDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]];
}

Eigen::VectorXd JackalTmpcAcadosSolver::getConstantVectorDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  return constants_.segment(
    constants_start_idc_[index], constants_sizes_[size_index]);
}

Eigen::MatrixXd JackalTmpcAcadosSolver::getConstantMatrixDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  const int rows = constants_sizes_[size_index];
  const int columns = constants_sizes_[size_index + 1];
  return Eigen::Map<const Eigen::MatrixXd>(
    constants_.data() + constants_start_idc_[index], rows, columns);
}

std::vector<int> JackalTmpcAcadosSolver::returnDataPositionsState(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), stage_vars_names_);
}

std::vector<int> JackalTmpcAcadosSolver::returnDataPositionsObjective(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_objectives_names_);
}

std::vector<int> JackalTmpcAcadosSolver::returnDataPositionsConstrain(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_constraints_names_);
}

void JackalTmpcAcadosSolver::initializeDefaultGuess()
{
  // Numerical initialization only.
  // Runtime feedback and PMPC references overwrite these values.
  // 0.0 m/s is inside the supplied reference-speed interval.
  constexpr double reference_speed = 0.0;

  stage_vars_.setZero();
  par_objectives_.setZero();

  // Stage-variable and reference ordering:
  // [a, omega, x, y, psi, v].
  for (int stage = 0; stage <= n_; ++stage)
  {
    const double reference_x =
      reference_speed * dt_ * static_cast<double>(stage);

    stage_vars_(nu_, stage) = reference_x;
    stage_vars_(nu_ + 3, stage) = reference_speed;

    par_objectives_(nu_, stage) = reference_x;
    par_objectives_(nu_ + 3, stage) = reference_speed;
  }

  last_updated_stage_vars_ = stage_vars_.col(0);

  const auto default_weights =
    hmpc_tmpc_settings::defaultWeights();

  par_weights_ = Eigen::Map<const Eigen::VectorXd>(
    default_weights.data(),
    static_cast<Eigen::Index>(default_weights.size()));

  if (n_par_constraints_ % 3 != 0)
  {
    throw std::runtime_error(
      "Jackal obstacle parameters must contain triples [a1, a2, b]");
  }

  par_constraints_.setZero();

  const int number_of_constraints = n_par_constraints_ / 3;

  for (int constraint = 0;
       constraint < number_of_constraints;
       ++constraint)
  {
    par_constraints_.row(3 * constraint).setConstant(1.0);
    par_constraints_.row(3 * constraint + 2).setConstant(100.0);
  }
}

void JackalTmpcAcadosSolver::loadStatesMatrixInSolver()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x = stage_vars_.block(kNu, stage, kNx, 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

    if (stage < n_)
    {
      Eigen::VectorXd u = stage_vars_.block(0, stage, kNu, 1);
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
    }
  }

  Eigen::VectorXd initial_state = stage_vars_.block(kNu, 0, kNx, 1);
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", initial_state.data());
}

void JackalTmpcAcadosSolver::updateMeasuredState()
{
  stage_vars_.col(0) = last_updated_stage_vars_;
}

void JackalTmpcAcadosSolver::updateMeasuredStateInHorizon()
{
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = last_updated_stage_vars_;
  }
}

void JackalTmpcAcadosSolver::shiftHorizonInMatrix(int time_shift)
{
  const int shift = std::max(0, std::min(time_shift, n_));
  const Eigen::MatrixXd previous = stage_vars_;
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = previous.col(std::min(stage + shift, n_));
  }
}

void JackalTmpcAcadosSolver::updateAcadosParameters()
{
  std::vector<double> parameters(kNumberOfAcadosParameters, 0.0);
  for (int stage = 0; stage <= n_; ++stage)
  {
    for (int index = 0; index < kNumberOfObjectiveParameters; ++index)
    {
      parameters[index] = par_objectives_(index, stage);
    }
    for (int index = 0; index < kNumberOfConstraintParameters; ++index)
    {
      parameters[kNumberOfObjectiveParameters + index] =
        par_constraints_(index, stage);
    }

    const int status = jackal_tmpc_acados_update_params(
      capsule_, stage, parameters.data(), kNumberOfAcadosParameters);
    if (status != 0)
    {
      throw std::runtime_error(
        "Updating acados parameters failed at stage " +
        std::to_string(stage) + " with status " + std::to_string(status));
    }
  }
}

void JackalTmpcAcadosSolver::loadObjectiveParams()
{
  updateAcadosParameters();
}

void JackalTmpcAcadosSolver::loadWeightsParams()
{
  // TMPC Q and R are compiled into the generated acados solver.
}

void JackalTmpcAcadosSolver::loadConstrainParams()
{
  updateAcadosParameters();
}

void JackalTmpcAcadosSolver::loadAllParameters()
{
  updateAcadosParameters();
}

void JackalTmpcAcadosSolver::resetSolver()
{
  if (jackal_tmpc_acados_reset(capsule_, 1) != 0)
  {
    throw std::runtime_error("Resetting the Jackal TMPC solver failed");
  }
  const Eigen::VectorXd current_weights = par_weights_;
  initializeDefaultGuess();
  par_weights_ = current_weights;
  loadAllParameters();
  loadStatesMatrixInSolver();
}

void JackalTmpcAcadosSolver::insertPredictedTrajectoryInMatrix()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x(kNx);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
    stage_vars_.block(kNu, stage, kNx, 1) = x;

    if (stage < n_)
    {
      Eigen::VectorXd u(kNu);
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
      stage_vars_.block(0, stage, kNu, 1) = u;
    }
  }

  // acados has no control variable at the terminal node. Reuse the final
  // optimized input to keep the legacy [u;x] warm-start matrix complete.
  stage_vars_.block(0, n_, kNu, 1) = stage_vars_.block(0, n_ - 1, kNu, 1);
}

int JackalTmpcAcadosSolver::mapAcadosStatus(int acados_status) const
{
  // Translate acados statuses to the original framework's exit codes.
  // Controller::controlLoop retries 0, -6 and -7, but not QP failure (-8).
  switch (acados_status)
  {
    case ACADOS_SUCCESS:
      return 1;
    case ACADOS_NAN_DETECTED:
      return -6;
    case ACADOS_MAXITER:
      return 0;
    case ACADOS_MINSTEP:
      return -7;
    case ACADOS_QP_FAILURE:
      return -8;
    default:
      return -7;
  }
}

int JackalTmpcAcadosSolver::runSolverStep()
{
  loadAllParameters();
  loadStatesMatrixInSolver();

  last_acados_status_ = jackal_tmpc_acados_solve(capsule_);
  if (last_acados_status_ != 0)
  {
    std::cerr
        << "\n[TMPC] acados status: "
        << last_acados_status_
        << "\n[TMPC] Initial state [x,y,psi,v]: "
        << stage_vars_.block(kNu, 0, kNx, 1).transpose()
        << "\n[TMPC] Reference rows [a,omega,x,y,psi,v]:\n"
        << par_objectives_
        << "\n[TMPC] Obstacle parameters:\n"
        << par_constraints_
        << "\n[TMPC] Solver statistics:\n"
        << std::flush;

    jackal_tmpc_acados_print_stats(capsule_);

    std::cout << std::flush;

    return mapAcadosStatus(last_acados_status_);
  }

  insertPredictedTrajectoryInMatrix();
  if (print_solver_data_)
  {
    printSolverData();
  }
  return 1;
}

void JackalTmpcAcadosSolver::printSolverData()
{
  double solve_time = std::numeric_limits<double>::quiet_NaN();
  int sqp_iterations = -1;
  ocp_nlp_get(nlp_solver_, "time_tot", &solve_time);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iterations);

  std::cout << "[MPC solver] acados status: " << last_acados_status_ << '\n'
            << "[MPC solver] SQP iterations: " << sqp_iterations << '\n'
            << "[MPC solver] solve time [ms]: " << 1000.0 * solve_time << '\n';
  if (print_solver_output_)
  {
    std::cout << "[MPC solver] [u;x] trajectory:\n" << stage_vars_ << std::endl;
  }
}

JackalPmpcAcadosSolver::JackalPmpcAcadosSolver()
  : SolverBase(
      hmpc_pmpc_settings::kDt,
      pmpc_data::kN,
      pmpc_data::kNx,
      pmpc_data::kNu,
      pmpc_data::kNx,
      hmpc_pmpc_settings::stageVariableNames(),
      hmpc_pmpc_settings::constantNames(),
      hmpc_pmpc_settings::constantDimensions(),
      hmpc_pmpc_settings::constantSizes(),
      hmpc_pmpc_settings::constantStartIndices(),
      hmpc_pmpc_settings::constantValues(),
      pmpc_data::kNumberOfAcadosParameters,
      hmpc_pmpc_settings::allParameterNames(),
      pmpc_data::kNumberOfObjectiveParameters,
      hmpc_pmpc_settings::objectiveParameterNames(),
      pmpc_data::kNumberOfWeightParameters,
      hmpc_pmpc_settings::weightParameterNames(),
      hmpc_pmpc_settings::kNDiscs,
      pmpc_data::kNumberOfConstraintParameters,
      hmpc_pmpc_settings::constraintParameterNames(),
      {"GoalOriented", "StaticPolyhedronConstraints"})
{
  capsule_ = jackal_pmpc_acados_create_capsule();
  if (capsule_ == nullptr)
  {
    throw std::runtime_error("Could not allocate Jackal PMPC acados capsule");
  }

  const int create_status = jackal_pmpc_acados_create(capsule_);
  if (create_status != 0)
  {
    jackal_pmpc_acados_free_capsule(capsule_);
    capsule_ = nullptr;
    throw std::runtime_error(
      "Could not create Jackal PMPC acados solver, status " +
      std::to_string(create_status));
  }

  nlp_config_ = jackal_pmpc_acados_get_nlp_config(capsule_);
  nlp_dims_ = jackal_pmpc_acados_get_nlp_dims(capsule_);
  nlp_in_ = jackal_pmpc_acados_get_nlp_in(capsule_);
  nlp_out_ = jackal_pmpc_acados_get_nlp_out(capsule_);
  nlp_solver_ = jackal_pmpc_acados_get_nlp_solver(capsule_);

  initializeDefaultGuess();
  loadAllParameters();
  loadStatesMatrixInSolver();
}

JackalPmpcAcadosSolver::~JackalPmpcAcadosSolver()
{
  if (capsule_ != nullptr)
  {
    jackal_pmpc_acados_free(capsule_);
    jackal_pmpc_acados_free_capsule(capsule_);
  }
}

int JackalPmpcAcadosSolver::constantIndexOrThrow(
  const std::string& constant_name) const
{
  const int index = returnDataPosition(constant_name, constants_names_);
  if (index < 0)
  {
    throw std::out_of_range("Unknown solver constant: " + constant_name);
  }
  return index;
}

bool JackalPmpcAcadosSolver::getConstantScalarBool(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]] != 0.0;
}

int JackalPmpcAcadosSolver::getConstantScalarInt(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return static_cast<int>(constants_[constants_start_idc_[index]]);
}

double JackalPmpcAcadosSolver::getConstantScalarDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  return constants_[constants_start_idc_[index]];
}

Eigen::VectorXd JackalPmpcAcadosSolver::getConstantVectorDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  return constants_.segment(
    constants_start_idc_[index], constants_sizes_[size_index]);
}

Eigen::MatrixXd JackalPmpcAcadosSolver::getConstantMatrixDouble(
  std::string constant_name)
{
  const int index = constantIndexOrThrow(constant_name);
  int size_index = 0;
  for (int item = 0; item < index; ++item)
  {
    size_index += constants_ndims_[item] <= 1 ? 1 : 2;
  }
  const int rows = constants_sizes_[size_index];
  const int columns = constants_sizes_[size_index + 1];
  return Eigen::Map<const Eigen::MatrixXd>(
    constants_.data() + constants_start_idc_[index], rows, columns);
}

std::vector<int> JackalPmpcAcadosSolver::returnDataPositionsState(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), stage_vars_names_);
}

std::vector<int> JackalPmpcAcadosSolver::returnDataPositionsObjective(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_objectives_names_);
}

std::vector<int> JackalPmpcAcadosSolver::returnDataPositionsConstrain(
  std::vector<std::string> find_names)
{
  return returnDataPositions(std::move(find_names), par_constraints_names_);
}

void JackalPmpcAcadosSolver::initializeDefaultGuess()
{
  // Numerical initialization only.
  // The runtime interface supplies the planning boundary state.
  stage_vars_.setZero();
  last_updated_stage_vars_ = stage_vars_.col(0);

  // Planar goal parameters: [goal_x, goal_y].
  par_objectives_.setZero();

  const auto default_weights =
    hmpc_pmpc_settings::defaultWeights();

  par_weights_ = Eigen::Map<const Eigen::VectorXd>(
    default_weights.data(),
    static_cast<Eigen::Index>(default_weights.size()));

  if (n_par_constraints_ % 3 != 0)
  {
    throw std::runtime_error(
      "Jackal obstacle parameters must contain triples [a1, a2, b]");
  }

  par_constraints_.setZero();

  const int number_of_constraints = n_par_constraints_ / 3;

  for (int constraint = 0;
       constraint < number_of_constraints;
       ++constraint)
  {
    par_constraints_.row(3 * constraint).setConstant(1.0);
    par_constraints_.row(3 * constraint + 2).setConstant(100.0);
  }
}

void JackalPmpcAcadosSolver::loadStatesMatrixInSolver()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x =
      stage_vars_.block(pmpc_data::kNu, stage, pmpc_data::kNx, 1);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());

    if (stage < n_)
    {
      Eigen::VectorXd u =
        stage_vars_.block(0, stage, pmpc_data::kNu, 1);
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
    }
  }

  Eigen::VectorXd initial_state =
    stage_vars_.block(pmpc_data::kNu, 0, pmpc_data::kNx, 1);
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", initial_state.data());
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", initial_state.data());
}

void JackalPmpcAcadosSolver::updateMeasuredState()
{
  stage_vars_.col(0) = last_updated_stage_vars_;
}

void JackalPmpcAcadosSolver::updateMeasuredStateInHorizon()
{
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = last_updated_stage_vars_;
  }
}

void JackalPmpcAcadosSolver::shiftHorizonInMatrix(int time_shift)
{
  const int shift = std::max(0, std::min(time_shift, n_));
  const Eigen::MatrixXd previous = stage_vars_;
  for (int stage = 0; stage < nbar_; ++stage)
  {
    stage_vars_.col(stage) = previous.col(std::min(stage + shift, n_));
  }
}

void JackalPmpcAcadosSolver::updateAcadosParameters()
{
  std::vector<double> parameters(
    pmpc_data::kNumberOfAcadosParameters, 0.0);
  for (int stage = 0; stage <= n_; ++stage)
  {
    for (int index = 0;
         index < pmpc_data::kNumberOfObjectiveParameters; ++index)
    {
      parameters[index] = par_objectives_(index, stage);
    }
    for (int index = 0; index < pmpc_data::kNumberOfWeightParameters; ++index)
    {
      parameters[pmpc_data::kNumberOfObjectiveParameters + index] =
        par_weights_(index);
    }
    for (int index = 0;
         index < pmpc_data::kNumberOfConstraintParameters; ++index)
    {
      parameters[
        pmpc_data::kNumberOfObjectiveParameters +
        pmpc_data::kNumberOfWeightParameters + index] =
        par_constraints_(index, stage);
    }

    const int status = jackal_pmpc_acados_update_params(
      capsule_, stage, parameters.data(),
      pmpc_data::kNumberOfAcadosParameters);
    if (status != 0)
    {
      throw std::runtime_error(
        "Updating PMPC acados parameters failed at stage " +
        std::to_string(stage) + " with status " + std::to_string(status));
    }
  }
}

void JackalPmpcAcadosSolver::loadObjectiveParams()
{
  updateAcadosParameters();
}

void JackalPmpcAcadosSolver::loadWeightsParams()
{
  updateAcadosParameters();
}

void JackalPmpcAcadosSolver::loadConstrainParams()
{
  updateAcadosParameters();
}

void JackalPmpcAcadosSolver::loadAllParameters()
{
  updateAcadosParameters();
}

void JackalPmpcAcadosSolver::resetSolver()
{
  if (jackal_pmpc_acados_reset(capsule_, 1) != 0)
  {
    throw std::runtime_error("Resetting the Jackal PMPC solver failed");
  }
  const Eigen::VectorXd current_weights = par_weights_;
  initializeDefaultGuess();
  par_weights_ = current_weights;
  loadAllParameters();
  loadStatesMatrixInSolver();
}

void JackalPmpcAcadosSolver::insertPredictedTrajectoryInMatrix()
{
  for (int stage = 0; stage <= n_; ++stage)
  {
    Eigen::VectorXd x(pmpc_data::kNx);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "x", x.data());
    stage_vars_.block(pmpc_data::kNu, stage, pmpc_data::kNx, 1) = x;

    if (stage < n_)
    {
      Eigen::VectorXd u(pmpc_data::kNu);
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, stage, "u", u.data());
      stage_vars_.block(0, stage, pmpc_data::kNu, 1) = u;
    }
  }

  // The Jackal terminal steady-state input is [a_T, omega_T] = [0, 0].
  // Store it in the framework's terminal [u; x] column.
  stage_vars_.block(0, n_, pmpc_data::kNu, 1).setZero();
}

int JackalPmpcAcadosSolver::mapAcadosStatus(int acados_status) const
{
  // Translate acados statuses to the original framework's exit codes.
  // Controller::controlLoop retries 0, -6 and -7, but not QP failure (-8).
  switch (acados_status)
  {
    case ACADOS_SUCCESS:
      return 1;
    case ACADOS_NAN_DETECTED:
      return -6;
    case ACADOS_MAXITER:
      return 0;
    case ACADOS_MINSTEP:
      return -7;
    case ACADOS_QP_FAILURE:
      return -8;
    default:
      return -7;
  }
}

int JackalPmpcAcadosSolver::runSolverStep()
{
  loadAllParameters();
  loadStatesMatrixInSolver();

  last_acados_status_ = jackal_pmpc_acados_solve(capsule_);
  if (last_acados_status_ != 0)
  {
    std::cerr
        << "\n[PMPC] acados status: "
        << last_acados_status_
        << "\n[PMPC] Initial state [x,y,psi,v]: "
        << stage_vars_.block(
               pmpc_data::kNu, 0, pmpc_data::kNx, 1).transpose()
        << "\n[PMPC] Goal parameters:\n"
        << par_objectives_
        << "\n[PMPC] Solver statistics:\n"
        << std::flush;

    jackal_pmpc_acados_print_stats(capsule_);

    std::cout << std::flush;

    return mapAcadosStatus(last_acados_status_);
  }

  insertPredictedTrajectoryInMatrix();
  if (print_solver_data_)
  {
    printSolverData();
  }
  return 1;
}

void JackalPmpcAcadosSolver::printSolverData()
{
  double solve_time = std::numeric_limits<double>::quiet_NaN();
  int sqp_iterations = -1;
  ocp_nlp_get(nlp_solver_, "time_tot", &solve_time);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iterations);

  std::cout << "[PMPC solver] acados status: " << last_acados_status_ << '\n'
            << "[PMPC solver] SQP iterations: " << sqp_iterations << '\n'
            << "[PMPC solver] solve time [ms]: " << 1000.0 * solve_time
            << '\n';
  if (print_solver_output_)
  {
    std::cout << "[PMPC solver] [u;x] trajectory:\n"
              << stage_vars_ << std::endl;
  }
}

}  // namespace mpc_solver
