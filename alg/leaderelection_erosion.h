/*
 * By Remco Surtel
 * Erosion-based leader election.
 * Based on the paper "Shape Formation by Programmable Particles"
 * From Giuseppe A. Di Luna, Paola Flocchini, Nicola Santoro, Giovanni Viglietta, Yukiko Yamauchi
 * https://arxiv.org/abs/1705.03538
 */

#ifndef AMOEBOTSIM_ALG_LEADERELECTION_EROSION_H_
#define AMOEBOTSIM_ALG_LEADERELECTION_EROSION_H_

#include <array>
#include <vector>

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

class LeaderElectionErosionParticle : public AmoebotParticle {
public:
    enum class State {
        Eligible,
        Candidate,
        Eroded,
        Leader,
        Finished
    };

    State state;

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, and a system which it belongs to.
    LeaderElectionErosionParticle(const Node head, const int globalTailDir,
        const int orientation, AmoebotSystem& system, State state);

    // Executes one particle activation.
    virtual void activate();

    // Functions for altering a particle's cosmetic appearance; headMarkColor
    // (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int headMarkColor() const;

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    LeaderElectionErosionParticle& nbrAtLabel(int label) const;

    // Returns the type of corner particle for the calling particle.
    int getCornerType() const;

    // Returns a count of the number of particle neighbors surrounding the calling
    // particle.
    int getNumberOfNbrs() const;
};

class LeaderElectionErosionSystem : public AmoebotSystem {
public:
    // Constructs a system of LeaderElectionErosionParticles with an optionally specified
    // size (#particles), and hole probability. holeProb in [0,1] controls how
    // "spread out" the system is; closer to 0 is more compressed, closer to 1 is
    // more expanded.
    LeaderElectionErosionSystem(int numParticles = 100);

    // Checks whether or not the system's run of the Leader Election algorithm has
    // terminated (all particles in state Finished or Leader).
    bool hasTerminated() const override;
};
#endif  // AMOEBOTSIM_ALG_LEADERELECTION_EROSION_H_