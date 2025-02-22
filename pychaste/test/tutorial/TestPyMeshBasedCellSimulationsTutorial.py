"""Copyright (c) 2005-2025, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# ifndef
# define TRIGGER_WIKI

## ## Introduction
## In this tutorial we show how Chaste can be used to create, run and visualize mesh-based simulations.
## Full details of the mathematical model can be found in van Leeuwen et al. (2009) [doi:10.1111/j.1365-2184.2009.00627.x].
##
## ## The Test

import unittest  # Python testing framework

import matplotlib.pyplot as plt  # Plotting
import numpy as np  # Matrix tools

import chaste  # The PyChaste module
import chaste.cell_based  # Contains cell populations
import chaste.mesh  # Contains meshes
import chaste.visualization  # Visualization tools

from chaste.cell_based import AbstractCellBasedTestSuite


class TestPyMeshBasedCellSimulationsTutorial(AbstractCellBasedTestSuite):

    ## ### Test 1 - a basic mesh-based simulation
    ## In the first test, we run a simple mesh-based simulation,
    ## in which we create a monolayer of cells, using a mutable mesh. Each cell is assigned a stochastic cell-cycle model.

    def test_monolayer(self):

        # JUPYTER_SETUP

        ## Next, we generate a mutable mesh. To create a `MutableMesh`, we can use the `HoneycombMeshGenerator`.
        ## This generates a honeycomb-shaped mesh, in which all nodes are equidistant. Here the first and second arguments define the size of the mesh -
        ## we have chosen a mesh that is 4 nodes (i.e. cells) wide, and 4 nodes high.

        chaste.core.OutputFileHandler("Python/TestMeshBasedCellSimulationsTutorial")
        generator = chaste.mesh.HoneycombMeshGenerator(4, 4)
        mesh = generator.GetMesh()

        ## Having created a mesh, we now create some cells. To do this, we use the `CellsGenerator` helper class,
        ## which is specialized by the type of cell cycle model required (here `UniformCellCycleModel`) and the dimension.
        ## For a list of possible cell cycle models see subclasses of `AbstractCellCycleModel`.
        ## Note that some of these models will require information on the surrounding medium such as Oxygen concentration to work,
        ## see specific class documentation for details. We create an empty vector of cells and pass this into the method along with the mesh.
        ## The second argument represents the size of that the list of cells should become - one cell for each node,
        ## the third argument specifies the proliferative type of the cell.

        transit_type = chaste.cell_based.TransitCellProliferativeType()
        cell_generator = chaste.cell_based.CellsGenerator["UniformCellCycleModel", 2]()
        cells = cell_generator.GenerateBasicRandom(mesh.GetNumNodes(), transit_type)

        ## Now we have a mesh and a set of cells to go with it, we can create a `CellPopulation`.
        ## In general, this class associates a collection of cells with a mesh. For this test, because we have a `MutableMesh`,
        ## we use a particular type of cell population called a `MeshBasedCellPopulation`.

        cell_population = chaste.cell_based.MeshBasedCellPopulation[2, 2](mesh, cells)

        ## To view the results of this and the next test in Paraview it is necessary to explicitly
        ## generate the required .vtu files.

        cell_population.AddPopulationWriterVoronoiDataWriter()

        ## We can set up a `VtkScene` to do a quick visualization of the population before running the analysis.

        scene = chaste.visualization.VtkScene[2]()
        scene.SetCellPopulation(cell_population)
        # JUPYTER_SHOW_FIRST
        scene.Start()  # JUPYTER_SHOW

        ## We then pass in the cell population into an `OffLatticeSimulation`, and set the output directory and end time.

        simulator = chaste.cell_based.OffLatticeSimulation[2, 2](cell_population)
        simulator.SetOutputDirectory("Python/TestMeshBasedCellSimulationsTutorial")
        simulator.SetEndTime(10.0)

        ## For longer simulations, we may not want to output the results every time step. In this case we can use the following method,
        ## to print results every 12 time steps instead. As the default time step used by the simulator is 30 seconds,
        ## this method will cause the simulator to print results every 6 minutes (or 0.1 hours).

        simulator.SetSamplingTimestepMultiple(12)

        ## We must now create one or more force laws, which determine the mechanics of the centres of each cell in a cell population.
        ## For this test, we use one force law, based on the spring based model, and pass it to the `OffLatticeSimulation`.
        ## For a list of possible forces see subclasses of `AbstractForce`. Note that some of these forces are not compatible with mesh-based simulations,
        ## see the specific class documentation for details. If you try to use an incompatible class then you will receive a warning.

        force = chaste.cell_based.GeneralisedLinearSpringForce[2, 2]()
        simulator.AddForce(force)

        ## Save snapshot images of the population during the simulation

        scene_modifier = chaste.cell_based.VtkSceneModifier[2]()
        scene_modifier.SetVtkScene(scene)
        scene_modifier.SetUpdateFrequency(100)
        simulator.AddSimulationModifier(scene_modifier)

        ## To run the simulation, we call `Solve()`. We can again do a quick rendering of the population at the end of the simulation

        scene.Start()
        simulator.Solve()
        scene.End()

        # JUPYTER_TEARDOWN

        ## Full results can be visualized in Paraview from the `file_handler.GetOutputDirectoryFullPath()` directory.

    ## ### Test 2 -  a basic mesh-based simulation with ghost nodes
    ## In the second test, we run a simple mesh-based simulation with ghost nodes, in which we create a monolayer of cells, using a mutable mesh.
    ## Each cell is assigned a stochastic cell-cycle model.

    def test_monolayer_with_ghost_nodes(self):

        # JUPYTER_SETUP

        ## We start by generating a mutable mesh. To create a `MutableMesh`, we can use the `HoneycombMeshGenerator` as before.
        ## Here the first and second arguments define the size of the mesh - we have chosen a mesh that is 2 nodes (i.e. cells) wide,
        ## and 2 nodes high. The third argument specifies the number of layers of ghost nodes to make.

        chaste.core.OutputFileHandler("Python/TestMeshBasedCellPopulationWithGhostNodes")
        generator = chaste.mesh.HoneycombMeshGenerator(5, 5, 2)
        mesh = generator.GetMesh()

        ## We only want to create cells to attach to real nodes, so we use the method `GetCellLocationIndices` to get the
        ## indices of the real nodes in the mesh. This will be passed in to the cell population later on.

        locs = generator.GetCellLocationIndices()

        ## Having created a mesh, we now create some cells. To do this, we use the `CellsGenerator` helper class again.
        ## This time the second argument is different and is the number of real nodes in the mesh.
        ## As before all cells have `TransitCellProliferativeType`.

        transit_type = chaste.cell_based.TransitCellProliferativeType()
        cell_generator = chaste.cell_based.CellsGenerator["UniformCellCycleModel", 2]()
        cells = cell_generator.GenerateBasicRandom(len(locs), transit_type)

        ## Now we have a mesh and a set of cells to go with it, we can create a `CellPopulation`.
        ## In general, this class associates a collection of cells with a set of elements or a mesh.
        ## For this test, because we have a `MutableMesh`, and ghost nodes we use a particular type of cell population called
        ## a `MeshBasedCellPopulationWithGhostNodes`. The third argument of the constructor takes a vector of the indices of the real nodes
        ## and should be the same length as the vector of cell pointers.

        cell_population = chaste.cell_based.MeshBasedCellPopulationWithGhostNodes[2](mesh, cells, locs)

        ## Again Paraview output is explicitly requested.

        cell_population.AddPopulationWriterVoronoiDataWriter()

        ## We can set up a `VtkScene` to do a quick visualization of the population before running the analysis.

        scene = chaste.visualization.VtkScene[2]()
        scene.SetCellPopulation(cell_population)
        scene.GetCellPopulationActorGenerator().SetShowVoronoiMeshEdges(True)
        # JUPYTER_SHOW

        ## We then pass in the cell population into an `OffLatticeSimulation`, and set the output directory, output multiple and end time.

        simulator = chaste.cell_based.OffLatticeSimulation[2, 2](cell_population)
        simulator.SetOutputDirectory("Python/TestMeshBasedCellPopulationWithGhostNodes")
        simulator.SetEndTime(10.0)
        simulator.SetSamplingTimestepMultiple(12)

        ## Save snapshot images of the population during the simulation

        scene_modifier = chaste.cell_based.VtkSceneModifier[2]()
        scene_modifier.SetVtkScene(scene)
        scene_modifier.SetUpdateFrequency(300)
        simulator.AddSimulationModifier(scene_modifier)

        ## Again we create a force law, and pass it to the `OffLatticeSimulation`.
        ## This force law ensures that ghost nodes don't exert forces on real nodes but real nodes exert forces on ghost nodes.

        force = chaste.cell_based.GeneralisedLinearSpringForce[2, 2]()
        simulator.AddForce(force)

        ## To run the simulation, we call `Solve()`.

        scene.Start()
        simulator.Solve()

        ## The next two lines are for test purposes only and are not part of this tutorial.
        ## If different simulation input parameters are being explored the lines should be removed.

        self.assertEqual(cell_population.GetNumRealCells(), 48)
        self.assertAlmostEqual(chaste.cell_based.SimulationTime.Instance().GetTime(), 10.0, 6)

        # JUPYTER_TEARDOWN

        ## Full results can be visualized in Paraview from the `file_handler.GetOutputDirectoryFullPath()` directory.


if __name__ == "__main__":
    unittest.main(verbosity=2)

# endif END_WIKI
