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
  // Draw points in between the edges connecting particles.
  virtual std::array<int, 6> borderPointBetweenEdgeColors() const;
  // Draw points as half circles when the node is shared by two particles.
  virtual std::array<int, 12> borderHalfPointBetweenEdgeColors() const;

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
    int destination;
  };

  // Used in stretch expansion to request a merge.
  struct MergeRequestToken : public LeaderElectionToken {
    MergeRequestToken(int origin = -1, int destination = -1) { this->origin = origin; this->destination = destination;}
  };
  // Used in stretch expansion to acknowledge a merge.
  struct MergeAckToken : public LeaderElectionToken {
    MergeAckToken(int origin = -1, int destination = -1) { this->origin = origin; this->destination = destination;}
  };
  // Used in stretch expansion to decline a merge.
  struct MergeNackToken : public LeaderElectionToken {
    MergeNackToken(int origin = -1, int destination = -1) { this->origin = origin; this->destination = destination;}
  };
  // Used to send the stretch count from the head to the tail node.
  struct CountToken : public LeaderElectionToken {
    int value;
    CountToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  // Used to send the adjacent stretch count from the tail back to the head node.
  struct CountReturnToken : public LeaderElectionToken {
    int value;
    CountReturnToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  // Used to attempt merge requests from stretches larger than 1.
  struct AttemptMergeToken : public LeaderElectionToken {
    int value;
    AttemptMergeToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  // Used to merge stretches larger than 1 and send the count to the head.
  struct MergeCountToken : public LeaderElectionToken {
    int value;
    MergeCountToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  // Used to attempt merge requests as a result of lexicographic comparison.
  struct LexCompAttemptMergeToken : public LeaderElectionToken {
    int value;
    LexCompAttemptMergeToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  /*
   * Lexicographic comparison tokens
   * 
   * LexCompInitToken : signals to clockwise adjacent stretch to initialize lexicographic comparison
   * LexCompAckToken : signals to counter-clockwise adjacent stretch that lexicographic comparison is initialized
   * LexCompNackToken : signals that counts are no longer equal; a merge has happened -> no lexicographic comparison
   * LexCompReqStretchLabelToken : request the next label from the clockwise adjacent stretch
   * LexCompReturnStretchLabelToken : returns the label of the next node to the counter-clockwise adjacent stretch
   * LexCompEndOfNbrStretchToken : signals to the initiating stretch that its adjacent stretch has exhausted all its labels
   * LexCompRetrieveNextLabelToken : retrieves the label of the next internal node of a stretch
   * LexCompNextLabelToken : returns the label of the next internal node of a stretch
   * LexCompEndOfStretchToken : signals to the head that all labels have been retrieved
   * LexCompRetrieveNextLabelForNbrToken : retrieves the label of the next internal node of a stretch for its neighbour
   * LexCompNextLabelForNbrToken : returns the label of the next internal node of a stretch for its neighbour
   * LexCompEndOfStretchForNbrToken : signals to the head that all labels have been retrieved for the neighbour
   * LexCompInterruptRightToken : interrupts the lexicographic comparison process with the clockwise adjacent stretch
   * LexCompInterruptLeftToken : interrupts the lexicographic comparison process with the counter-clockwise adjacent stretch
   * LexCompCleanUpToken : sent from head to tail to reset 'retrieved' flags
   * LexCompCleanUpForNbrToken : sent from head to tail to reset 'retrievedForNbr' flags
   */
  struct LexCompToken : public Token {
    // origin is used to define the direction (label) that a LeaderElectionToken
    // was received from.
    int origin;
    int destination;
  };
  struct LexCompInitToken : public LexCompToken {
    int value;
    LexCompInitToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  struct LexCompAckToken : public LexCompToken {
    LexCompAckToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompNackToken : public LexCompToken {
    LexCompNackToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompReqStretchLabelToken : public LexCompToken {
    LexCompReqStretchLabelToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompReturnStretchLabelToken : public LexCompToken {
    int value;
    LexCompReturnStretchLabelToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  struct LexCompEndOfNbrStretchToken : public LexCompToken {
    LexCompEndOfNbrStretchToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompRetrieveNextLabelToken : public LexCompToken {
    LexCompRetrieveNextLabelToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompNextLabelToken : public LexCompToken {
    int value;
    LexCompNextLabelToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  struct LexCompEndOfStretchToken : public LexCompToken {
    LexCompEndOfStretchToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompRetrieveNextLabelForNbrToken : public LexCompToken {
    LexCompRetrieveNextLabelForNbrToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompNextLabelForNbrToken : public LexCompToken {
    int value;
    LexCompNextLabelForNbrToken(int origin = -1, int value = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->value = value;
    }
  };
  struct LexCompEndOfStretchForNbrToken : public LexCompToken {
    LexCompEndOfStretchForNbrToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompInterruptRightToken : public LexCompToken {
    LexCompInterruptRightToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompInterruptLeftToken : public LexCompToken {
    LexCompInterruptLeftToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompCleanUpToken : public LexCompToken {
    LexCompCleanUpToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  struct LexCompCleanUpForNbrToken : public LexCompToken {
    LexCompCleanUpForNbrToken(int origin = -1, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
    }
  };
  /*
   * Termination detection tokens
   * 
   * TerminationDetectionToken : sent by the initiator of termination detection
   * TerminationDetectionReturnToken : sent back to the initiator
   */
  struct TerminationDetectionToken : public LeaderElectionToken {
    int counter;
    int ttl;
    int traversed;
    TerminationDetectionToken(int origin = -1, int counter = 0, int ttl = 0, int traversed = 0, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->counter = counter;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
  struct TerminationDetectionReturnToken : public LeaderElectionToken {
    int counter;
    int ttl;
    int traversed;
    bool termination;
    TerminationDetectionReturnToken(int origin = -1, int counter = 0, int ttl = 0, int traversed = 0, bool termination = false, int destination = -1) {
      this->origin = origin;
      this->destination = destination;
      this->counter = counter;
      this->ttl = ttl;
      this->traversed = traversed;
      this->termination = termination;
    }
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

    bool nextNodeClone = false; 
    bool prevNodeClone = false;
    bool cloneChange = false;

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

    // Used to remember that the count has been sent to the tail node.
    bool countSent = false;

    // Used by head node to denote whether it has sent a token to initialize lexicographic comparison
    bool lexCompInit = false;

    // Used for lexicographic comparison to denote whether a merge attempt should be made at the end
    bool lexCompTryMerge = false;

    // Used by head node to denote whether it is currently participating in lexicographic comparison
    bool lexicographicComparisonLeft = false;

    // Used by head node to denote whether it is currently participating in lexicographic comparison
    bool lexicographicComparisonRight = false;

    // Used by head node during lexicographic comparison to denote whether it has requested the next label from the adjacent stretch
    bool requestedNbrLabel = false;

    // Used by head node during lexicographic comparison to denote whether it has received the requested label from the adjacent stretch
    bool receivedNbrLabel = false;

    // Used to store the label sent by the adjacent stretch during lexicographic comparison
    int NbrLabel = 0;

    // Used by head node during lexicographic comparison to denote whether it has requested the next internal label
    bool requestedLabel = false;

    // Used by head node during lexicographic comparison to denote whether it has received the requested internal label
    bool receivedLabel = false;

    // Used by head node during lexicographic comparison to denote whether its counter-clockwise adjacent stretch has requested a label
    bool receivedLabelRequestFromNbr = false;

    // Used by head node during lexicographic comparison to denote whether it has requested the next internal label
    bool requestedLabelForNbr = false;

    // Used by head node during lexicographic comparison to denote whether it has received the requested internal label
    bool receivedLabelForNbr = false;
    
    // Used to store internal labels during lexicographic comparison
    int internalLabel = 0;

    // Used to store internal labels during lexicographic comparison
    int internalLabelForNbr = 0;

    // Used by head node during lexicographic comparison
    // Set to 1 if this stretch had the first larger node, -1 if it was the other stretch
    // As long as labels are equal, remains 0
    int firstLargerLabel = 0;

    // Denotes whether this node's label has been retrieved for lexicographic comparison
    bool retrieved = false;

    // Denotes whether this node's label has been retrieved for lexicographic comparison
    bool retrievedForNbr = false;

    // Termination detection variables
    bool terminationDetectionInitiated = false;

    // Executes one node activation.
    virtual void activate();

    // Used to reset variables and remove tokens for lexicographic comparison
    void lexCompCleanUp();
    void lexCompCleanUpForNbr();

    // Function to draw a virtual node which is emulated by a particle.
    void paintNode(const int color);

    // Methods for passing, taking, and checking the ownership of tokens at the
    // node level
    template <class TokenType>
    bool hasNodeToken(int dir, bool checkClone=true) const;
    template <class TokenType>
    std::shared_ptr<TokenType> peekNodeToken(int dir, bool checkClone=true) const;
    template <class TokenType>
    std::shared_ptr<TokenType> takeNodeToken(int dir, bool checkClone=true);
    template <class TokenType>
    void passNodeToken(int dir, std::shared_ptr<TokenType> token, bool checkClone=true);
    LeaderElectionNode* nextNode(bool recursion=false) const;
    LeaderElectionNode* prevNode(bool recursion=false) const;
  };

  protected:
   std::vector<LeaderElectionNode*> nodes;
   std::array<int, 18> borderColorLabels;
   std::array<int, 6> borderPointColorLabels;
   std::array<int, 6> borderPointBetweenEdgeColorLabels;
   std::array<int, 12> borderHalfPointBetweenEdgeColorLabels;
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