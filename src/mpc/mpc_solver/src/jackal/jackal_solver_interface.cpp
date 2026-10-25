#include <mpc_solver/jackal/jackal_solver_interface.h>

#include "acados/acados_solver.h"

std::unique_ptr<SolverBase> returnSolverPointer(int solver_number)
{
  if (solver_number == 0)
  {
    return std::unique_ptr<SolverBase>(
      new mpc_solver::JackalTmpcAcadosSolver());
  }

  if (solver_number == 1)
  {
    return std::unique_ptr<SolverBase>(
      new mpc_solver::JackalPmpcAcadosSolver());
  }

  return nullptr;
}

std::vector<std::unique_ptr<SolverBase>>
returnSolverPointersVector()
{
  std::vector<std::unique_ptr<SolverBase>> solvers;

  solvers.emplace_back(
    new mpc_solver::JackalTmpcAcadosSolver());

  solvers.emplace_back(
    new mpc_solver::JackalPmpcAcadosSolver());

  return solvers;
}