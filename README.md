# Graphics Project - Cloth Simulator

## Project Overview

This project uses a mass-spring particle system to construct a physically-based cloth simulation. It simulates the
behavior and look of cloth, taking into account both internal and external forces, collisions with different
objects, and realistic physics-based shading.

## Please reach out to me directly if you'd like to see the code.

I completed this project as part of a course so for academic integrity reasons, I don't want to share it or make it 
publicly available, but upon request I am happy to explore the code with anyone if they are interested.

## How to Run

Provide the `simulation_number` as a single command line argument to run the corresponding simulation. If you don't
provide a command line argument, it will default to simulation 1:

* 1: Generated cloth mesh draped over a sphere and interacting with the floor - this is the BEST simulation

  ![gif-of-simulation1](screenshot.gif)

* 2: Generated cloth mesh draped over a sphere defined by an arbitrary input .obj file - works pretty well

  ![gif-of-simulation2](screenshot1.gif)

* 3: Generated cloth mesh draped over a cube defined by an arbitrary input .obj file - works pretty well

  ![gif-of-simulation3](screenshot2.gif)

* 4: Generated cloth mesh draped over a table defined by an arbitrary input .obj file (the table might not be visible in
  the scene, but you should see the cloth drape over part of the shape of a table and then slide off)
* 5: Input cloth mesh draped over a sphere defined by an arbitrary input .obj file. Input cloth mesh files work nowhere
  near as well as when we generate the cloth mesh, so this simulation does not work great, but you can see that it is
  capable of loading an input .obj file for the cloth, rendering it and applying cloth dynamics to it.
* 6: Generated cloth mesh draped over a pyramid defined by an arbitrary input .obj file (haven't been able to get
  the angles and setup right for this yet so that everything appears in the scene)

## Objectives Completed

1. **Integrate cloth material handling into rendering pipeline**

2. **Create a data structure to store a mass-spring particle system used to simulate internal cloth dynamics**

3. **Compute forces and apply Velocity Verlet to mass-spring particle system to simulate internal cloth dynamics**

4. **Apply strain limiting to prevent excessive cloth deformation**

7. **Add friction force to the surface of the cloth to simulate more natural behaviour**

9. **Apply final velocity updates for simulation time step using central time differencing methods**

10. **Implement physics based shading of cloth materials**

#### Additional Achievement:

- Added collision detection and response for spheres, floors, and arbitrary triangle meshes using SAH BVH for
  efficiency in detecting collisions with arbitrary triangle meshes.

Note: The numbering of objectives follows the original proposal. Objectives 5, 6, and 8 from the original proposal which
involved robust collision handling of the cloth with itself to allow folding effects according to the methods described
in [2] were not implemented as I was unable to get them working at all with the time I had. I pivoted to using simpler
approaches similar to what is described in [1] rather than the ones described in [2] that also provide folding behaviour
in order to get a final project that functions correctly in time for the deadline. In the future, a nice extension to
this project would be revisiting these objectives in order to get the cloth to have correct self intersection handling,
so it can exhibit folding behaviour as well.

## Technical Outline

1. **Cloth Material Integration into Rendering Pipeline**:
    - Extended the existing Material class to include a new MAT_CLOTH type.
    - Created a ClothParticle class to represent individual particles in the cloth mesh, storing position, velocity,
      acceleration, normal, texture coordinates, mass, grid position and the rest length of springs connecting it to
      neighbor particles in the 2D grid on each ClothParticle instance.
    - Created a ClothParticleSystem class to manage the entire cloth simulation, including initialization, stepping,
      force computation, constraint handling, and strain limiting. The initialization supports the generation of a cloth
      mesh from scratch or enables a cloth mesh to be loaded from a .obj file.

2. **Mass-Spring Particle System Data Structure**:
    - Used a 2D vector (std::vector<std::vector<ClothParticle>>) to represent the grid of particles similar to what
      is described in [1].
    - Used an unordered_map to store the mapping between each vertex in the cloth mesh and the corresponding
      ClothParticle instance in the grid. This ensures that when we update the cloth mesh based on the updated state of
      all the particles in the 2D grid, we can easily figure out which vertex to update in the mesh based on the state
      of it's corresponding ClothParticle instance.This map must be kept up to date and thus must be completely updated
      after each update to the cloth mesh for each simulation step.
    - Used an unordered_map to store rest lengths between particles, allowing for efficient lookup of spring
      rest lengths for the springs connecting each of the neighboring particles when computing spring forces. This was
      necessary to support loading of arbitrary cloth meshes where the rest lengths are not known ahead of time.
    - Created methods to initialize the particle system, including setting up initial positions, velocities, and rest
      lengths for structural, shear, and bending springs.

3. **Force Computation and Velocity Verlet Integration**:
    - Used Hooke's law with damping to compute spring forces in the computeSpringForce function as described in [1].
    - By looking up the rest lengths in the unordered_map, we can compute the spring forces between neighboring
      particles in the grid and the current particle.
    - Also made sure to apply the computed force in both directions to both particles for each spring.
    - Applied gravity and spring forces in the computeForces method of ClothParticleSystem to update the acceleration of
      all particles to reflect the forces applied to them.
    - Implemented Velocity Verlet integration in the step function for the first half of the time step:
        * Update particle positions: x(t+Δt) = x(t) + v(t)Δt + 0.5a(t)Δt^2
        * Compute forces and update accelerations of all particles
        * Update particle velocities first half: v(t+Δt) = v(t) + 0.5[a(t) + a(t+Δt)]Δt

4. **Strain Limiting**:
    - Created the applyStrainLimiting function in ClothParticleSystem.
    - Used an iterative approach to limit both strain and strain rate where we iterate over all particles for a fixed
      number of iterations using a hybrid approach similar to what is described in [1] and [2]:
        * Compute current spring lengths and compare to rest lengths for all neighbors of each particle.
        * Apply corrective displacements to particles to satisfy strain constraints. I used a strain limit of 10% in the
          final results. This is where the strain limiting is applied and follows a similar but simpler method for
          strain limiting described in [1], where we just update the positions so that they now satisfy the strain
          limit constraints set by applying corrective displacements if they don't already satisfy them.
        * Update velocities based on position changes to approximate strain rate limiting. This attempts to use a
          similar approach to strain rate limiting described in [2] where corrective impulses are applied to the
          velocity. I have used a simpler approach where I just update the velocity based on the position updates that
          have been made in the strain limiting.
    - All strain limiting updates are applied to both the particle and its neighbor with the required update being
      split between the two.

7. **Friction Force Implementation**:
    - Added friction handling in collision response functions (handleCollisionWithSphereSurface,
      handleCollisionWithFloor, handleCollisionWithObstacle) by reducing the tangential velocity after collision to
      simulate friction.
    - Used different friction coefficients for different surfaces to simulate realistic sliding and wrinkle formation
      when the cloth interacts with different surfaces like when it is interacting with both the sphere and the floor
      for example, a higher coefficient of friction is used for the floor than the surface of the sphere.

9. **Final Velocity Updates using Central Time Differencing**:
    - Implemented in the step function of ClothParticleSystem, although Central Time Differencing was not used in the
      final implementation.
    - Instead, after applying all forces and constraints, the final velocity updates are made using Velocity Verlet at
      the end of the simulation step.
      v(t+Δt) = v(t) + 0.5[a(t) + a(t+Δt)]Δt

10. **Physics-Based Cloth Shading**:
    - Implemented the CharlieD BRDF in the Material#BRDF function and used it in the shadeRasterizer function for
      handling MAT_CLOTH materials.
    - Added roughness and color properties to the Material class for cloth-specific shading.
    - Computed the BRDF value accounting for roughness to make the surface of the cloth have a rougher surface rather
      than just using the Lambertian BRDF.

#### Additional Implementations:

- **Collision Detection and Response**:
    * Implemented handleCollisionWithSphereSurface, handleCollisionWithFloor, and handleCollisionWithObstacle functions
      in the ClothParticle class.
    * Used a SAH BVH for efficient collision detection with arbitrary meshes.
    * Added collision response by adjusting particle positions and velocities to resolve collisions and applying
      friction as well as simulating sliding along the surface by only keeping the tangential acceleration and velocity
      components.

The implementation of each of these technical components works together to create a comprehensive cloth simulation
system. The system allows for realistic cloth behavior under various conditions, including gravity, friction, and
collisions with different types of objects while maintaining stability using strain limiting.

## Implementation Details

The `ClothParticle` class represents individual particles in the cloth:

```cpp
class ClothParticle {
public:
    float3 position;
    float3 velocity;
    float3 acceleration;
    // Other properties
    // ...

    // Methods for initializing and updating particles
    // ...
};
```

The `ClothParticleSystem` manages the entire cloth simulation and contains a majority of the implementation:

```cpp
class ClothParticleSystem {
public:
    std::vector<std::vector<ClothParticle>> particles;
    TriangleMesh particlesMesh;
    // Other properties
    // ...

    void initialize();
    void step(float dt);
    void computeForces();
    void applyStrainLimiting(float deltaTime, float maxStrainRatio, float minStrainRatio, int iterations);
    // Other simulation methods
    // ...
};
```

Cloth physics-based shading is implemented using the CharlieD BRDF, based on the Microfacet Sheen BRDF [3] with the
handling of cloth materials for shading defined in the shadeRasterizer function and the implementation of the CharlieD
BRDF defined in the BRDF method of the Material class.

## References

1. Provot, X. (1995). Deformation constraints in a mass-spring model to describe rigid cloth behavior. In Graphics
   interface (pp. 147-154).
2. Bridson, R., Fedkiw, R., & Anderson, J. (2002). Robust treatment of collisions, contact and friction for cloth
   animation. ACM Transactions on Graphics (TOG), 21(3), 594-603.
3. Estevez, A., & Kulla, C. (2017). Production Friendly Microfacet Sheen BRDF. Technical Report, Sony Pictures
   Imageworks.
