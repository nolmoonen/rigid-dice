`rigid-dice` is a 3D rigid body simulation based on the Physically Based Modeling course notes by David Baraff. 
It supports a simulation of convex polyhedrons which collide and form resting contact. The simulation is rendered
in an OpenGL powered GLFW window.

_In this project I have not been able to achieve a stable implementation of resting contact resolution. In an attempt to 
remedy this, the error correction method proposed in Baraff's 1995 article on Interactive Rigid Body Simulation[3] is 
implemented, but to no avail. While this makes the simulation unstable, my hope is that it may serve as a learning resource for 
those attempting to undertake the same project._

While multiple versions exist of Baraff's course notes 
([1995](http://www.cs.cmu.edu/~baraff/pbm/pbm.html),
[1997](http://www.cs.cmu.edu/~baraff/sigcourse/index.html),
[1999](https://graphics.stanford.edu/courses/cs448b-00-winter/papers/phys_model.pdf),
[2001](http://graphics.pixar.com/pbm2001/)), I have simply selected the most recent version, which can be obtained from Pixar's 
website. I would like to note that the differences in these versions are small, especially in the sections I will make 
remarks about later on. 

### Table of contents
* **Collision detection and contact derivation** explanation of how collision detection is done, and how contact 
information is derived from the collision.
* **Notes on _Physically Based Modeling_** notes on potential mistakes I noticed in these course notes.
* **Implementation notes** some design philosophy of the project.
* **Sources** sources containing the course notes and other algorithms used in the project.
* **Building** instructions for building the project.
* **Image credit**

### Collision detection and contact derivation
An earlier iteration of this project used a combination of GJK and EPA to find the triangle in CSO space that lies 
closest to the origin. From this triangle the topological elements where extracted for both rigid bodies involved. These
topological elements where expanded to the largest parallel topological element based on [5].

Later on I realised that this novel approach is not compatible with the contact derivation described in Baraff's course
notes. In these notes, contact regions are formed by the points on a body which are within the distance threshold from the 
separating plane. Due to this, a simpler separating plane method was adapted, where contacts are derived exactly as laid 
out in [1] and [4]. This however resulted in a ridiculously intricate contact derivation algorithm. There is however
no way around this if one for example wants to find all intersection points between face-face contact.

### Notes on _Physically Based Modeling_
After having carefully studied the course notes, it seems to me that some errors persist in these notes. Listed in order
of appearance:

#### Dot product in total torque computation
On page G54 of [1] the following code is listed to apply a contact force to rigid bodies `A` and `B`. Throughout the
document, the convention is followed that `*` denotes a dot product and `^` denotes a cross product.
```cpp
    /* apply the force ‘f n’ positively to A... */
    A->force += f * n;
    A->torque += (contacts[i].p - A->x) * (f*n);
    /* and negatively to B */
    B->force -= f * n;
    B->torque -= (contacts[i].p - B->x) * (f*n);
``` 
It seems to me that a cross product should be used to compute the torque contribution. This is in line with Equation
(2-23) on page G11 of [1].
#### Faulty edge-edge normal derivative code
On page G64 of [1] the following code is listed to compute the derivative of the normal vector. Again, it is stated that `*` denotes a 
dot product and `^` denotes a cross product.
```cpp
/* return the derivative of the normal vector */
triple computeNdot(Contact *c)
{
    if(c->vf) /* vertex/face contact */
    {
        /* The vector 'n' is attached to B, so... */
        return c->b->omega ^ c->n;
    }
    else
    {
        /* This is a little trickier. The unit normal 'n' is
        \hat{n} = \frac{(e_a \times e_b)}{||e_a \times e_b||}
        Differentiating \hat{n} with respect to time is left
        as an exercise... but here's some code */
        triple eadot = c->a->omega ^ ea, /* \dot{e}_a */
               ebdot = c->b->omega ^ eb; /* \dot{e}_b */
               n1 = ea * eb,
               z = eadot * eb + ea * ebdot;
        double l = length(n1);
        n1 = n1 / length; /* normalize */
        return (z - ((z * n) * n)) / l;
    }
}
```
Something odd is going on with the code listing for the derivative of the normal for
edge-edge contacts. `n1` and `z` are missing a type, furthermore `n` and `length` are not declared. 
After having made the derivation, which can be found in `doc/derivation.pdf`, the correct listing should be:
```cpp
triple computeNdot(Contact *c) {
    if (c->vf) {
        return c->b->omega ^ c->n;
    } else {
        triple eadot = c->a->omega ^ ea,
               ebdot = c->b->omega ^ eb,
               n1 = ea ^ eb,
               z = eadot ^ eb + ea ^ ebdot;
        double l = length(n1);
        n1 = n1 / l;
        return (z - ((z * n1) . n1)) / l;
    }
}
```
`.` is used for clarity to denote vector-scalar multiplication. Special thanks to [Tijn Bertens](https://github.com/TijnBertens)
for helping me derive this formula.

#### Discrepancy between intersection detection
[1] describes collision detection as follows: when at time `t_0`, an interpenetration is detected at time `t_0 + dt`
the time of collision `t_c` is found within margin `[-e, e]`. The system is now restarted at time `t_0 + t_c`.
However, when at `t_c` the bodies penetrate with a certain value `[-e, 0]` the system will in the next iteration be
in a state of interpenetration, violating the assumption that it is not. To circumvent this issue, the interpenetration
detection routine should report two bodies as penetrating when their separation distance is at least `-e`. This
can be achieved by translating one body `e` away from the other.

#### Computing the torque exerted by contacts on bodies
On page G66 of [1] the following code is listed to compute the force and torque exerted by contact `j` on body `A`.
```cpp
/* What force and torque does contact j exert on body A? */
triple force_on_a = 0,
torque_on_a = 0;
if(cj.a == ci.a)
{
    /* force direction of jth contact force on A */
    force_on_a = nj;
    /* torque direction */
    torque_on_a = (pj - A->x) ^ nj;
}
else if(cj.b == ci.a)
{
    force_on_a = - nj;
    torque_on_a = (pj - A->x) ^ nj;
}
```
However, it seems to me that the torque computation should also take into account the negation of the force. Not doing
this (and copying the code as is), can lead to an asymmetric matrix **A**. Having made this observation, the code should be:
```cpp
triple force_on_a = 0,
torque_on_a = 0;
if (cj.a == ci.a) {
    force_on_a = nj;
    torque_on_a = (pj - A->x) ^ force_on_a;
} else if (cj.b == ci.a) {
    force_on_a = - nj;
    torque_on_a = (pj - A->x) ^ force_on_a;
}
```
The same change needs to be made in the computation to calculate the force and torque exerted on body `B` by contact `j`.

#### Implementation notes
`simulation` is a self-contained simulation engine and should not interact with the rendering/window system which is 
`system`. `shape` contains a description of a shape that may be interpreted as a hitbox for collision detection, as well 
as a mesh for rendering. 

As [3] suggests in the paragraph "Ambiguous collisions", testing for interpenetration depth is very error prone. 
This problem can be circumvented by first testing for intersection where a pair of bodies are translated `e` towards
each other, and then testing for intersection there a pair of bodies are translated `e` away from each other. If the
first test results in an intersection, we know there is an interpenetration. If the first does not result in an 
intersection but the second does, we know there is contact. If both test do not indicate an intersection, we know that 
the pair of bodies are not in contact.
 
#### Sources
1. David Baraff, Physically Based Modeling: Rigid Body Simulation, [link](http://graphics.pixar.com/pbm2001/), [permanent link](https://web.archive.org/web/20200223171751/http://graphics.pixar.com/pbm2001/).
2. David Baraff, Fast contact force computation for nonpenetrating rigid bodies, [10.1145/192161.192168](https://doi.org/10.1145/192161.192168)
3. David Baraff, Interactive simulation of solid rigid bodies, [10.1109/38.376615](https://doi.org/10.1109/38.376615).
4. David Baraff, Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies, [10.1145/74333.74356](https://doi.org/10.1145/74333.74356)
5. Lixin Zhang, Jing Xiao, Derivation of contact states from geometric models of objects, [10.1109/ISATP.1995.518797](http://dx.doi.org/10.1109/ISATP.1995.518797)

#### Building
*   Clone [GLFW 3.3.2](https://github.com/glfw/glfw/releases/tag/3.3.2) into directory `external/glfw-3.3.1`.
*   Clone [glad v0.1.33](https://github.com/Dav1dde/glad/releases/tag/v0.1.33) into directory `external/glad-0.1.33`.
*   Clone [GLM 0.9.9.8](https://github.com/g-truc/glm/releases/tag/0.9.9.8) into directory `external/glm-0.9.9.8`.
*   Clone [GSL 2.5.0](https://github.com/ampl/gsl/releases/tag/v2.5.0) (from a CMake mirror) into 
    directory `external/gsl-2.5.0`.
*   Clone [stb](https://github.com/nothings/stb) into directory `external/stb`.
*   Build using CMake.
 
#### Image credit
* HDRI obtained from [HdriHaven](https://hdrihaven.com/), and converted into a cube-mapped png using 
 [HDRI-to-CubeMap](https://matheowis.github.io/HDRI-to-CubeMap/).
* The table texture is obtained from [TextureHaven](https://texturehaven.com/).
* All other textures are created by me.