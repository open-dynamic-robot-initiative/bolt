/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <bolt/bolt.hpp>

namespace py = pybind11;
using namespace bolt;

PYBIND11_MODULE(bolt, m)
{
    py::class_<Bolt>(m, "Bolt")
        .def(py::init<>())
        .def("initialize", &Bolt::initialize)
        .def("acquire_sensors", &Bolt::acquire_sensors)
        .def("send_target_joint_torque",
             &Bolt::send_target_joint_torque,
             py::arg("target_joint_torque"))
        .def("get_motor_board_errors", &Bolt::get_motor_board_errors)
        .def("get_motor_board_enabled", &Bolt::get_motor_board_enabled)
        .def("get_motor_enabled", &Bolt::get_motor_enabled)
        .def("get_motor_ready", &Bolt::get_motor_ready)
        .def("get_joint_positions", &Bolt::get_joint_positions)
        .def("get_joint_velocities", &Bolt::get_joint_velocities);
}
