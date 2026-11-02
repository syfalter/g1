// Narrow OMPL adapter. Python owns the existing collision scope and all deadlines.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ompl/config.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace py = pybind11;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class SeededSampler : public ob::RealVectorStateSampler {
public:
    SeededSampler(const ob::StateSpace* space, std::uint32_t seed)
        : ob::RealVectorStateSampler(space) { rng_.setLocalSeed(seed); }
};

class SeededConnect : public og::RRTConnect {
public:
    SeededConnect(const ob::SpaceInformationPtr& si, std::uint32_t seed)
        : og::RRTConnect(si) { rng_.setLocalSeed(seed); }
};

std::vector<std::vector<double>> solve(
    const std::vector<double>& start, const std::vector<double>& goal,
    const std::vector<double>& lower, const std::vector<double>& upper,
    const py::function& valid, const py::function& stop, std::uint32_t seed) {
    const auto n = start.size();
    if (!n || n > 32 || goal.size() != n || lower.size() != n || upper.size() != n)
        throw std::invalid_argument("Invalid joint route dimensions");
    auto space = std::make_shared<ob::RealVectorStateSpace>(n);
    ob::RealVectorBounds bounds(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(start[i]) || !std::isfinite(goal[i]) ||
            !std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i] ||
            start[i] < lower[i] || start[i] > upper[i] || goal[i] < lower[i] || goal[i] > upper[i])
            throw std::invalid_argument("Invalid guarded joint route endpoints/bounds");
        bounds.setLow(i, lower[i]);
        bounds.setHigh(i, upper[i]);
    }
    space->setBounds(bounds);
    space->setStateSamplerAllocator([seed](const ob::StateSpace* s) {
        return std::make_shared<SeededSampler>(s, seed ^ 0x51a9u);
    });
    auto values = [n](const ob::State* state) {
        const auto* data = state->as<ob::RealVectorStateSpace::StateType>()->values;
        return std::vector<double>(data, data + n);
    };
    auto si = std::make_shared<ob::SpaceInformation>(space);
    std::exception_ptr callback_error;
    si->setStateValidityChecker([&](const ob::State* state) {
        if (callback_error) return false;
        try { return valid(values(state)).cast<bool>(); }
        catch (...) { callback_error = std::current_exception(); return false; }
    });
    // Candidate edges only; the original timed dense validator remains mandatory.
    si->setStateValidityCheckingResolution(std::min(0.5, 0.02 / space->getMaximumExtent()));
    si->setup();
    ob::ScopedState<> a(space), b(space);
    for (std::size_t i = 0; i < n; ++i) { a[i] = start[i]; b[i] = goal[i]; }
    auto problem = std::make_shared<ob::ProblemDefinition>(si);
    problem->setStartAndGoalStates(a, b, 1e-9);
    auto planner = std::make_shared<SeededConnect>(si, seed);
    planner->setRange(0.25);
    planner->setProblemDefinition(problem);
    planner->setNearestNeighbors<ompl::NearestNeighborsLinear>();
    // Let OMPL unwind its search normally, then propagate cancellation unchanged.
    ob::PlannerTerminationCondition termination([&]() {
        if (callback_error) return true;
        try { return stop().cast<bool>(); }
        catch (...) { callback_error = std::current_exception(); return true; }
    });
    planner->solve(termination);
    if (callback_error) std::rethrow_exception(callback_error);
    if (!problem->hasExactSolution()) return {};
    auto path = std::dynamic_pointer_cast<og::PathGeometric>(problem->getSolutionPath());
    if (!path) throw std::runtime_error("OMPL returned a non-geometric path");
    std::vector<std::vector<double>> result;
    for (auto* state : path->getStates()) result.push_back(values(state));
    return result;
}

PYBIND11_MODULE(_task_ompl_route, module) {
    module.attr("ompl_version") = OMPL_VERSION;
    module.def("solve", &solve, py::arg("start"), py::arg("goal"), py::arg("lower"),
               py::arg("upper"), py::arg("valid"), py::arg("stop"), py::arg("seed") = 19);
}
