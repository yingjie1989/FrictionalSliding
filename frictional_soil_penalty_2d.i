#  This is a benchmark test that checks constraint based frictional
#  contact using the penalty method.  In this test a constant
#  displacement is applied in the horizontal direction to simulate
#  a small block come sliding down a larger block.
#
#  A friction coefficient of 0.4 is used.  The gold file is run on one processor
#  and the benchmark case is run on a minimum of 4 processors to ensure no
#  parallel variability in the contact pressure and penetration results.
#

[Mesh]
  file = 2_blocks_2d.e
  patch_size = 1
[]

[GlobalParams]
  displacements = 'disp_x disp_y'
[]

[Variables]
  [./disp_x]
  [../]
  [./disp_y]
  [../]
#  [./disp_z]
#  [../]
[]

[AuxVariables]
  [./penetration]
  [../]
  [./vel_x]
  [../]
  [./accel_x]
  [../]
  [./vel_y]
  [../]
  [./accel_y]
  [../]
#  [./inc_slip_x]
#  [../]
#  [./inc_slip_y]
#  [../]
#  [./accum_slip_x]
#  [../]
#  [./accum_slip_y]
#  [../]
[]


[Kernels]
  [./TensorMechanics]
    displacements = 'disp_x disp_y'
  [../]
  [./inertia_x]
    type = InertialForce
    variable = disp_x
    velocity = vel_x
    acceleration = accel_x
    beta = 0.25
    gamma = 0.5
    eta=0.0
  [../]
  [./inertia_y]
    type = InertialForce
    variable = disp_y
    velocity = vel_y
    acceleration = accel_y
    beta = 0.25
    gamma = 0.5
    eta=0.0
  [../]
  [./gravity_z]
    type = Gravity
    variable = disp_y
    value = -9.81
  [../]
[]



[AuxKernels]
#  [./zeroslip_x]
#    type = ConstantAux
#    variable = inc_slip_x
#    boundary = 3
#    execute_on = timestep_begin
#    value = 0.0
#  [../]
#  [./zeroslip_y]
#    type = ConstantAux
#    variable = inc_slip_y
#    boundary = 3
#    execute_on = timestep_begin
#    value = 0.0
#  [../]
#  [./accum_slip_x]
#    type = AccumulateAux
#    variable = accum_slip_x
#    accumulate_from_variable = inc_slip_x
#    execute_on = timestep_end
#  [../]
#  [./accum_slip_y]
#    type = AccumulateAux
#    variable = accum_slip_y
#    accumulate_from_variable = inc_slip_y
#    execute_on = timestep_end
#  [../]
  [./penetration]
    type = PenetrationAux
    variable = penetration
    boundary = 3
    paired_boundary = 2
  [../]
  [./accel_x]
    type = NewmarkAccelAux
    variable = accel_x
    displacement = disp_x
    velocity = vel_x
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./vel_x]
    type = NewmarkVelAux
    variable = vel_x
    acceleration = accel_x
    gamma = 0.5
    execute_on = timestep_end
  [../]
  [./accel_y]
    type = NewmarkAccelAux
    variable = accel_y
    displacement = disp_y
    velocity = vel_y
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./vel_y]
    type = NewmarkVelAux
    variable = vel_y
    acceleration = accel_y
    gamma = 0.5
    execute_on = timestep_end
  [../]
[]

#[Postprocessors]
#  [./nonlinear_its]
#    type = NumNonlinearIterations
#    execute_on = timestep_end
#  [../]
#  [./penetration]
#    type = NodalVariableValue
#    variable = penetration
#    nodeid = 222
#  [../]
#  [./contact_pressure]
#    type = NodalVariableValue
#    variable = contact_pressure
#    nodeid = 222
#  [../]
#[]

[BCs]
  [./slave_left_x]
    type = NeumannBC
    variable = disp_x
    boundary = 9
    value = 1.29e6
  [../]
  [./bottom_x]
    type = PresetBC
    variable = disp_x
    boundary = 1
    value = 0.0
  [../]
  [./bottom_y]
    type = PresetBC
    variable = disp_y
    boundary = 1
    value = 0.0
  [../]
  [./left_x]
    type = PresetBC
    variable = disp_x
    boundary = 5
    value = 0.0
  [../]
 [./right_x]
    type = PresetBC
    variable = disp_x
    boundary = 6
    value = 0.0
  [../]
[]

[Materials]
  [./Elasticity_tensor_soil]
    type = ComputeIsotropicElasticityTensor
    block = 1
    youngs_modulus = 1.3e9
    poissons_ratio = 0.45
  [../]
  [./Elasticity_tensor_concrete]
    type = ComputeIsotropicElasticityTensor
    block = 2
    youngs_modulus = 2e10
    poissons_ratio = 0.25
  [../]
  [./strain_soil]
    type = ComputeSmallStrain
    block = 1
    displacements = 'disp_x disp_y'
  [../]
  [./stress_soil]
    type = ComputeLinearElasticStress
    block = 1
  [../]

  [./strain_concrete]
    type = ComputeSmallStrain
    block = 2
    displacements = 'disp_x disp_y'
  [../]
  [./stress_concrete]
    type = ComputeLinearElasticStress
    block = 2
  [../]
  [./density_concrete]
    type = GenericConstantMaterial
    block = 2
    prop_names = 'density'
    prop_values = '2400'
  [../]
  [./density_soil]
    type = GenericConstantMaterial
    block = 1
    prop_names = 'density'
    prop_values = '2000'
  [../]
[]

#[Preconditioning]
#  [./SMP]
#    type = SMP
#    full = true
#  [../]
#[]



[Executioner]
  type = Transient
  solve_type = 'PJFNK'

#  petsc_options_iname = '-pc_type -sub_pc_type -pc_asm_overlap -ksp_gmres_restart'
#  petsc_options_value = 'asm     lu    20    101'

  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu     superlu_dist'  

  line_search = 'none'

  l_max_its = 100
  nl_max_its = 1000
  dt = 0.01
  end_time = 1.0
  num_steps = 1000
  l_tol = 1e-6
  nl_rel_tol = 1e-10
  nl_abs_tol = 1e-6
  dtmin = 1e-5
  [./Predictor]
    type = SimplePredictor
    scale = 1.0
  [../]
[]

[Outputs]
  file_base = frictional_concretesoil2d_out
  interval = 1
  [./exodus]
    type = Exodus
    elemental_as_nodal = true
  [../]
  [./console]
    type = Console
    max_rows = 5
  [../]
[]

[Contact]
  [./leftright]
    slave = 3
    master = 2
    model = coulomb
    penalty = 1e+7
    friction_coefficient = 0.15
    formulation = tangential_penalty
    system = constraint
#    normal_smoothing_distance = 0.1
  [../]
[]
