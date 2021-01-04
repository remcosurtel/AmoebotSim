/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * S-contraction leader election.
 * Based on the paper "Distributed Leader Election and Computation of Local Identifiers
 *                     for Programmable Matter"
 * By Nicolas Gastineau, Wahabou Abdou, Nader Mbarek, and  Olivier Togni.
 * https://arxiv.org/abs/1807.10461 */

#ifndef AMOEBOTSIM_ALG_LEADERELECTION_S_CONTRACTION_H_
#define AMOEBOTSIM_ALG_LEADERELECTION_S_CONTRACTION_H_

#include <array>
#include <vector>

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

using namespace std;

class LeaderElectionSContractionParticle : public AmoebotParticle {
public:
  enum class State {
    Candidate,
    NotElected,
    Leader
  };

  State state;

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, and a system which it belongs to.
  LeaderElectionSContractionParticle(const Node head, const int globalTailDir,
                                const int orientation, AmoebotSystem &system,
                                State state);

  // Executes one particle activation.
  virtual void activate();

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
  LeaderElectionSContractionParticle &nbrAtLabel(int label) const;

  // Returns the string to be displayed when this particle is inspected; used
  // to snapshot the current values of this particle's memory at runtime.
  virtual QString inspectionText() const;

  // Returns true iff this particle is S-contractible
  bool isSContractible();

  // Returns true iff the candidates in this particle's neighborhood are connected
  bool candidatesConnected();

  // Returns true iff this particle is adjacent to a node which is not occupied
  // By a particle in the Candidate state
  bool nonCandidateAdjacent();

  // Returns true iff this particle has a candidate neighbor
  bool hasCandidateNbr();

protected:
  // The LeaderElectionToken struct provides a general framework of any token
  // under the General Leader Election algorithm.
  struct LeaderElectionToken : public Token {
    // origin is used to define the direction (label) that a LeaderElectionToken
    // was received from.
    int origin;
  };
};

class LeaderElectionSContractionSystem : public AmoebotSystem {
public:
  // Constructs a system of LeaderElectionDeterministicParticles with an optionally
  // specified size (#particles).
  LeaderElectionSContractionSystem(int numParticles = 100, QString fileName = "");

  string outputPath = "";

  // Checks whether or not the system's run of the Leader Election algorithm has
  // terminated (all particles in state Finished or Leader).
  bool hasTerminated() const override;
};
#endif // AMOEBOTSIM_ALG_LEADERELECTION_S_CONTRACTION_H_