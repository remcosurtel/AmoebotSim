/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Stationary and deterministic leader election.
 * Based on the paper "Stationary and Deterministic Leader Election in 
 * Self-organizing Particle Systems"
 * By Rida A. Bazzi and Joseph L. Briones.
 * DOI: 10.1007/978-3-030-34992-9_3 */

#ifndef AMOEBOTSIM_ALG_LEADERELECTION_STATIONARY_DETERMINISTIC_H_
#define AMOEBOTSIM_ALG_LEADERELECTION_STATIONARY_DETERMINISTIC_H_

#include <array>
#include <vector>

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

using namespace std;

class LeaderElectionStationaryDeterministicParticle : public AmoebotParticle {
public:
  enum class State {
    IdentificationLabeling,
    StretchExpansion,
    Demoted,
    Candidate,
    Leader,
    Finished
  };

  State state;

  int currentNode;

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, and a system which it belongs to.
  LeaderElectionStationaryDeterministicParticle(const Node head, const int globalTailDir,
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
  LeaderElectionStationaryDeterministicParticle &nbrAtLabel(int label) const;

  // Returns the string to be displayed when this particle is inspected; used
  // to snapshot the current values of this particle's memory at runtime.
  virtual QString inspectionText() const;

  // Returns the borderColors and borderPointColors arrays associated with the
  // particle to draw the boundaries for leader election.
  virtual std::array<int, 18> borderColors() const;
  virtual std::array<int, 6> borderPointColors() const;

  // Returns the label associated with the direction which the next (resp.
  // previous) node is according to the cycle that the node is on (which is
  // determined by the provided nodeDir parameter).
  // Negative values represent nodes of the same particle.
  // int getNextNodeDir(const int nodeDir) const;
  // int getPrevNodeDir(const int nodeDir) const;

  // Returns a count of the number of particle neighbors surrounding the calling
  // particle.
  int getNumberOfNbrs() const;

protected:
  // The LeaderElectionToken struct provides a general framework of any token
  // under the General Leader Election algorithm.
  struct LeaderElectionToken : public Token {
    // origin is used to define the direction (label) that a LeaderElectionToken
    // was received from.
    int origin;
  };

  // Used in stretch expansion to request a merge.
  struct MergeRequestToken : public LeaderElectionToken {
    MergeRequestToken(int origin = -1) { this->origin = origin; }
  };
  // Used in stretch expansion to acknowledge a merge.
  struct MergeAckToken : public LeaderElectionToken {
    MergeAckToken(int origin = -1) { this->origin = origin; }
  };
  // Used in stretch expansion to decline a merge.
  struct MergeNackToken : public LeaderElectionToken {
    MergeNackToken(int origin = -1) { this->origin = origin; }
  };

 private:
  friend class LeaderElectionStationaryDeterministicSystem;

  // The nested class LeaderElectionNode is used to define the behavior for the
  // nodes as described in the paper
  class LeaderElectionNode {
   public:
    enum class SubPhase {
      Initial = 0,
      Stuff
    };

    LeaderElectionNode();

    /* General variables in node memory:
     * From the particle's perspective, this node is on the border with the 
     * particles in local directions nodeDir and (nodeDir + 5) % 6. 
     * The neighboring particle emulating the next (respectively, previous)
     * node on this node's boundary is in local direction nextnodeDir
     * (respectively, prevnodeDir). -1 if next node is on this particle. */
    int nodeDir, nextNodeDir, prevNodeDir;

    State nodeState;
    SubPhase subPhase;
    LeaderElectionStationaryDeterministicParticle* particle;

    /* This variable represents the label of the node.
     * Nodes are assigned a unary label, +1 for border nodes that belong to 
     * only one particle, -1 for border nodes that are shared between particles. */
    int unaryLabel;

    /* This variable is maintained by the leader of a stretch. It is equal to
     * the sum of the labels of the nodes in the stretch. It never exceeds 6. */
    int count;

    /* Indicates whether there is a pending merge request. */
    bool mergePending = false;
    bool mergeAck = false;

    /* Indicates the direction of the pending merge. 
     * -1: merge with stretch to the left (previous nodes) 
     * +1: merge with stretch to the right (next nodes) */
    int mergeDir = 0;

    /* These variables represent the parent and child nodes of this node in
     * the stretch that this node belongs to. Initially, -1 represents that
     * this node has no parent or child yet. These values will be updated
     * during the stretch expansion phase. */
    LeaderElectionNode* predecessor = nullptr;
    LeaderElectionNode* successor = nullptr;

    // Executes one node activation.
    virtual void activate();

    // Function to draw a virtual node which is emulated by a particle.
    void paintNode(const int color);

    // Methods for passing, taking, and checking the ownership of tokens at the
    // node level
    template <class TokenType>
    bool hasNodeToken(int dir) const;
    template <class TokenType>
    std::shared_ptr<TokenType> peekNodeToken(int dir) const;
    template <class TokenType>
    std::shared_ptr<TokenType> takeNodeToken(int dir);
    template <class TokenType>
    void passNodeToken(int dir, std::shared_ptr<TokenType> token);
    LeaderElectionNode* nextNode() const;
    LeaderElectionNode* prevNode() const;
  };

  protected:
   std::vector<LeaderElectionNode*> nodes;
   std::array<int, 18> borderColorLabels;
   std::array<int, 6> borderPointColorLabels;
};

class LeaderElectionStationaryDeterministicSystem : public AmoebotSystem {
public:
  // Constructs a system of LeaderElectionStationaryDeterministicParticles with an optionally
  // specified size (#particles), and hole probability. holeProb in [0,1]
  // controls how "spread out" the system is; closer to 0 is more compressed,
  // closer to 1 is more expanded.
  LeaderElectionStationaryDeterministicSystem(int numParticles = 100);

  // Checks whether or not the system's run of the Leader Election algorithm has
  // terminated (all particles in state Finished or Leader).
  bool hasTerminated() const override;
};
#endif // AMOEBOTSIM_ALG_LEADERELECTION_STATIONARY_DETERMINISTIC_H_