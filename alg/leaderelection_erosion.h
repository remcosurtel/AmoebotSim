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

using namespace std;

class LeaderElectionErosionParticle : public AmoebotParticle {
public:
    enum class State {
        None,
        Eligible,
        Candidate,
        Eroded,
        Root,
        Tree,
        Leader,
        Finished
    };

    State state;

    int parent;

    set <int> children;

    int cornerType;

    bool stateStable;

    bool stable;

    bool treeDone;

    bool chooseTokenSent = false;

    int numCandidates = 0;

    set <int> candidates;

    bool sameHandedness = false;

    bool hasMoved = false;

    int numNbrsCandidate = -1;

    // Constructs a new particle with a node position for its head, a global
    // compass direction from its head to its tail (-1 if contracted), an offset
    // for its local compass, and a system which it belongs to.
    LeaderElectionErosionParticle(const Node head, const int globalTailDir,
        const int orientation, AmoebotSystem& system, State state, int cornerType,
        bool stable, bool stateStable, bool treeDone);

    // Executes one particle activation.
    virtual void activate();

    // Check if the calling particle is 'locked'.
    // A particle is locked iff it is a 3-corner particle and
    // its middle eligible neighbour is also a 3-corner particle.
    bool isLocked() const;

    // Functions for altering a particle's cosmetic appearance; headMarkColor
    // (respectively, tailMarkColor) returns the color to be used for the ring
    // drawn around the head (respectively, tail) node. Tail color is not shown
    // when the particle is contracted. headMarkDir returns the label of the port
    // on which the black head marker is drawn.
    virtual int headMarkColor() const;

    // Returns the local directions from the head (respectively, tail) on which to
    // draw the direction markers. Intended to be overridden by particle
    // subclasses, as the default implementations return -1 (no markers).
    virtual int headMarkDir() const;

    // Gets a reference to the neighboring particle incident to the specified port
    // label. Crashes if no such particle exists at this label; consider using
    // hasNbrAtLabel() first if unsure.
    LeaderElectionErosionParticle& nbrAtLabel(int label) const;

    // Returns the type of corner particle for the calling particle.
    int getCornerType() const;

    // Returns a count of the number of particle neighbors surrounding the calling
    // particle.
    int getNumberOfNbrs() const;

    // Update the 'stable' flag by checking neighbouring particles.
    void updateStability();

    // Checks if the treeDone flag should be set.
    bool treeIsDone() const;

    // Determines the number of adjacent candidate particles.
    // Note that particles in state 'Root' are also candidates.
    int getNumCandidates();

protected:
    // The LeaderElectionToken struct provides a general framework of any token
    // under the General Leader Election algorithm.
    struct LeaderElectionToken : public Token {
        // origin is used to define the direction (label) that a LeaderElectionToken
        // was received from.
        int origin;
    };

    // Used in tree formation to signal the parent particle.
    struct ParentToken : public LeaderElectionToken {
        ParentToken(int origin = -1) {
            this->origin = origin;
        }
    };

    // Handedness agreement tokens.
    struct YouChooseToken : public LeaderElectionToken {
        YouChooseToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct YouDoNotChooseToken : public LeaderElectionToken {
        YouDoNotChooseToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct ChosenToken : public LeaderElectionToken {
        ChosenToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct NotChosenToken : public LeaderElectionToken {
        NotChosenToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct SameHandednessToken : public LeaderElectionToken {
        SameHandednessToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct IAmEliminatedToken : public LeaderElectionToken {
        IAmEliminatedToken(int origin = -1) {
            this->origin = origin;
        }
    };
    struct YouAreEliminatedToken : public LeaderElectionToken {
        YouAreEliminatedToken(int origin = -1) {
            this->origin = origin;
        }
    };
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