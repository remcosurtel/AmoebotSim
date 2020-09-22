/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Erosion-based leader election.
 * Based on the paper "Shape Formation by Programmable Particles"
 * By Giuseppe A. Di Luna, Paola Flocchini, Nicola Santoro, Giovanni Viglietta,
 * Yukiko Yamauchi https://arxiv.org/abs/1705.03538 */

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
    RootElection,
    Leader
  };

  State state;

  // Denotes the local direction to the parent particle.
  int parent;

  // Set with 1 integer for each child particle
  // Each integer denotes the local direction to the child particle
  set<int> children;

  // Used by candidate particles to store the latest encoding.
  string currentEncoding = "";

  // Denotes whether this particle has already sent its own
  // neighborhood encoding.
  bool nbrhdEncodingSent = false;

  // Used by candidate particles to remember whether they have sent
  // their latest encoding to the other candidates.
  bool encodingSent = false;

  // Denotes whether this particle has sent an encoding request
  // to one of its children.
  bool sentEncodingRequest = false;

  // Denotes whether this particle has exhausted its entire
  // subtree by retrieving neighborhood encodings from all
  // particles.
  bool treeExhausted = false;

  // Set of integers representing local directions to child particles.
  // This is a subset of the set 'children'.
  // Each child in this set has exhausted its entire subtree such that
  // 'treeExhausted' is set to true.
  set<int> childrenExhausted;

  // Denotes whether this particle is a corner particle, and if so
  // what type of corner particle it is.
  // x-corner particles have 0 <= x <= 3 adjacent eligible neighbors.
  // -2 indicates that the value has not yet been calculated.
  // -1 indicates that the particle is not a corner particle.
  int cornerType = -2;

  // Indicates whether the state of this particle has been stable for
  // at least one round.
  bool stateStable = false;

  // Denotes whether the neighborhood of this particle is stable.
  // i.e., stateStable == true for each neighbor of this particle.
  bool stable = false;

  // Used in the spanning forest formation phase to denote whether
  // tree formation has finished.
  bool treeDone = false;

  // Used to remember whether a choose tokens have been sent.
  bool chooseTokenSent = false;

  // Used by candidate particles to denote the number of other
  // candidates in the system.
  // I.e., in a system with 2 candidates, each candidate will
  // set this variable to 1.
  int numCandidates = 0;

  // Set of integers with local directions to the other candidates.
  set<int> candidates;

  // Denotes whether the particle has agreed on its handedness
  // with its parent / the other candidates.
  // This value is set to true by default since we assume
  // common chirality for all particles.
  bool sameHandedness = true;

  // Used in the handedness agreement phase to remember
  // whether this particle has moved.
  bool hasMoved = false;

  // Used in the handedness agreement phase to denote the number
  // of occupied common neighbors for two candidates.
  int numNbrsCandidate = -1;

  // Used in the handedness agreement phase to denote whether
  // this particle was chosen.
  bool notChosen = false;

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, and a system which it belongs to.
  LeaderElectionErosionParticle(const Node head, const int globalTailDir,
                                const int orientation, AmoebotSystem &system,
                                State state);

  // Executes one particle activation.
  virtual void activate();

  // Check if the calling particle is 'locked'.
  // A particle is locked iff it is a 3-corner particle and
  // its middle eligible neighbor is also a 3-corner particle.
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
  LeaderElectionErosionParticle &nbrAtLabel(int label) const;

  // Returns the string to be displayed when this particle is inspected; used
  // to snapshot the current values of this particle's memory at runtime.
  virtual QString inspectionText() const;

  // Returns the type of corner particle for the calling particle.
  int getCornerType() const;

  // Returns a count of the number of particle neighbors surrounding the calling
  // particle.
  int getNumberOfNbrs() const;

  // Update the 'stable' flag by checking neighboring particles.
  void updateStability();

  // Checks if the treeDone flag should be set.
  bool treeIsDone() const;

  // Determines the number of adjacent candidate particles.
  // Note that particles in state 'Root' are also candidates.
  int getNumCandidates();

  // Returns the neighborhood encoding for this particle.
  string getNeighborhoodEncoding();

  // Sends the given encoding to the other candidates.
  void sendEncodingCandidates(string encoding);

  // Sends the given encoding to the parent particle.
  void sendEncodingParent(string encoding);

  // Sends a SubTreeExhaustedToken in the given direction.
  void sendExhaustedToken(int dir);

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
    ParentToken(int origin = -1) { this->origin = origin; }
  };

  // Handedness agreement tokens.
  struct YouChooseToken : public LeaderElectionToken {
    YouChooseToken(int origin = -1) { this->origin = origin; }
  };
  struct YouDoNotChooseToken : public LeaderElectionToken {
    YouDoNotChooseToken(int origin = -1) { this->origin = origin; }
  };
  struct ChosenToken : public LeaderElectionToken {
    ChosenToken(int origin = -1) { this->origin = origin; }
  };
  struct NotChosenToken : public LeaderElectionToken {
    NotChosenToken(int origin = -1) { this->origin = origin; }
  };
  struct SameHandednessToken : public LeaderElectionToken {
    SameHandednessToken(int origin = -1) { this->origin = origin; }
  };
  struct IAmEliminatedToken : public LeaderElectionToken {
    IAmEliminatedToken(int origin = -1) { this->origin = origin; }
  };
  struct YouAreEliminatedToken : public LeaderElectionToken {
    YouAreEliminatedToken(int origin = -1) { this->origin = origin; }
  };
  struct EncodingTokenCandidate : public LeaderElectionToken {
    string encoding;
    EncodingTokenCandidate(int origin = -1, string encoding = "") {
      this->origin = origin;
      this->encoding = encoding;
    }
  };
  struct RequestEncodingToken : public LeaderElectionToken {
    RequestEncodingToken(int origin = -1) { this->origin = origin; }
  };
  struct EncodingToken : public LeaderElectionToken {
    string encoding;
    EncodingToken(int origin = -1, string encoding = "") {
      this->origin = origin;
      this->encoding = encoding;
    }
  };
  struct SubTreeExhaustedToken : public LeaderElectionToken {
    SubTreeExhaustedToken(int origin = -1) { this->origin = origin; }
  };
};

class LeaderElectionErosionSystem : public AmoebotSystem {
public:
  // Constructs a system of LeaderElectionErosionParticles with an optionally
  // specified size (#particles), and hole probability. holeProb in [0,1]
  // controls how "spread out" the system is; closer to 0 is more compressed,
  // closer to 1 is more expanded.
  LeaderElectionErosionSystem(int numParticles = 100);

  // Checks whether or not the system's run of the Leader Election algorithm has
  // terminated (all particles in state Finished or Leader).
  bool hasTerminated() const override;
};
#endif // AMOEBOTSIM_ALG_LEADERELECTION_EROSION_H_