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
    TreeFormation,
    TreeComparison,
    Candidate,
    Leader,
    Finished
  };

  State state;

  // Variables for "trees to break symmetry" phase

  // Stores the direction to the next particle of the stretch
  // Used to communicate with other candidates
  int nextDirCandidate;
  // Stores the current number of remaining candidates
  int numCandidates = 0;
  // Stores the count of the head node for candidate particles
  int headCount = 0;
  // Denotes whether this particle is part of a tree
  bool tree = false;
  // Denotes whether the subtree rooted at this particle is fully formed
  bool treeDone = false;
  // Denotes the local direction to the parent particle.
  int parent = -1;
  // Set with 1 integer for each child particle
  // Each integer denotes the local direction to the child particle
  set<int> children;
  // Set when childTokens are sent from candidate to avoid duplicates
  bool childTokensSent = false;
  // Set when TreeComparisonStartTokens and TreeFormationDoneTokens are sent
  bool treeFormationDone = false;
  // Stores the number of received treeFormationFinishedTokens
  int treeFormationFinishedTokensReceived = 0;
  // Denotes whether tree comparison can start
  bool treeComparisonReady = false;
  // Denotes whether this particle has already sent its own neighbourhood encoding
  bool nbrhdEncodingSentRight = false;
  bool nbrhdEncodingSentLeft = false;
  // Stores the current neighbourhood encoding
  string currentEncodingRight = "";
  string currentEncodingLeft = "";
  string currentEncodingNbr = "";
  // Denotes whether a request for a new encoding has been received
  bool nbrEncodingRequestReceived = false;
  // Denotes whether a new encoding has been requested
  bool encodingRequestedRight = false;
  bool encodingRequestedLeft = false;
  bool nbrEncodingRequested = false;
  // Denotes whether the requested encoding has been received
  bool encodingReceivedRight = false;
  bool encodingReceivedLeft = false;
  bool nbrEncodingReceived = false;
  // Denotes whether this particle has exhausted its entire subtree 
  // by retrieving neighborhood encodings from all particles.
  bool treeExhaustedRight = false;
  bool treeExhaustedLeft = false;
  bool nbrTreeExhausted = false;
  // Set of integers representing local directions to child particles.
  // This is a subset of the set 'children'.
  // Each child in this set has exhausted its entire subtree such that
  // 'treeExhausted' is set to true.
  set<int> childrenExhaustedRight;
  set<int> childrenExhaustedLeft;
  // Denotes the number of comparisonTokens from other candidates that have been received
  int comparisonsReceived = 0;
  // Denotes whether this particle has finished its comparison to the adjacent candidate
  bool comparisonDone = false;
  // Stores the result of a comparison
  // -1 for smaller, 0 for equal, 1 for larger
  int comparisonResult = 0;
  // Denotes whether the result of the current comparison has been sent
  bool comparisonSent = false;
  // Stores the results of tree comparison for all candidates
  // In clockwise direction, starting from the comparison of this candidate
  // to its right neighbour
  std::vector<int> comparisonResults = {};

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

  // Returns the neighborhood encoding for this particle.
  string getNeighborhoodEncoding();

  // Used in tree comparison to determine the direction in which tokens should be forwarded
  // so that they are passed along the boundary
  int getNextDir(int prevDir);

  // Calculates maximal non-descending sequence(s) of candidates ordered in clockwise direction
  // Used in the tree comparison phase to eliminate candidates
  set<std::vector<int>> getMaxNonDescSubSeq(std::vector<int> input);

protected:
  // The LeaderElectionToken struct provides a general framework of any token
  // under the General Leader Election algorithm.
  struct LeaderElectionToken : public Token {
    // origin is used to define the direction (label) that a LeaderElectionToken
    // was received from.
    int origin;
    int destination;
  };

  // Used in tree formation to signal the parent particle.
  struct ParentToken : public LeaderElectionToken {
    ParentToken(int origin = -1) { this->origin = origin; }
  };
  // Used in tree formation to signal the child particle to join the tree
  struct ChildToken : public LeaderElectionToken {
    ChildToken(int origin = -1) { this->origin = origin; }
  };
  // Used to signal to childNodes that the comparison phase is going to begin
  struct TreeComparisonStartToken : public LeaderElectionToken {
    TreeComparisonStartToken(int origin = -1) { this->origin = origin; }
  };
  // Used to signal to candidates that the tree formation phase is finished
  struct TreeFormationFinishedToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    TreeFormationFinishedToken(int origin = -1, int ttl = 0, int traversed = 0) {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
  struct ComparisonResultToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    int result;
    ComparisonResultToken(int origin = -1, int ttl = 0, int traversed = 0, int result = 0) {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
      this->result = result;
    }
  };
  struct RequestCandidateEncodingToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    RequestCandidateEncodingToken(int origin = -1, int ttl = 0, int traversed = 0) {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
  struct CandidateEncodingToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    string encoding;
    CandidateEncodingToken(int origin = -1, int ttl = 0, int traversed = 0, string encoding = "") {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
      this->encoding = encoding;
    }
  };
  struct CandidateTreeExhaustedToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    CandidateTreeExhaustedToken(int origin = -1, int ttl = 0, int traversed = 0) {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
  struct RequestEncodingRightToken : public LeaderElectionToken {
    RequestEncodingRightToken(int origin = -1) { this->origin = origin; }
  };
  struct RequestEncodingLeftToken : public LeaderElectionToken {
    RequestEncodingLeftToken(int origin = -1) { this->origin = origin; }
  };
  struct EncodingRightToken : public LeaderElectionToken {
    string encoding;
    EncodingRightToken(int origin = -1, string encoding = "") {
      this->origin = origin;
      this->encoding = encoding;
    }
  };
  struct EncodingLeftToken : public LeaderElectionToken {
    string encoding;
    EncodingLeftToken(int origin = -1, string encoding = "") {
      this->origin = origin;
      this->encoding = encoding;
    }
  };
  struct SubTreeExhaustedRightToken : public LeaderElectionToken {
    SubTreeExhaustedRightToken(int origin = -1) { this->origin = origin; }
  };
  struct SubTreeExhaustedLeftToken : public LeaderElectionToken {
    SubTreeExhaustedLeftToken(int origin = -1) { this->origin = origin; }
  };
  struct CleanUpToken : public LeaderElectionToken {
    CleanUpToken(int origin = -1) { this->origin = origin; }
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
   std::vector<LeaderElectionNode*> nodes = {};
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
  LeaderElectionStationaryDeterministicSystem(int numParticles = 100, QString fileName = "");

  string outputPath = "";

  // Checks whether or not the system's run of the Leader Election algorithm has
  // terminated (all particles in state Finished or Leader).
  bool hasTerminated() const override;
};
#endif // AMOEBOTSIM_ALG_LEADERELECTION_STATIONARY_DETERMINISTIC_H_