/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Deterministic leader election.
 * Based on the paper "Deterministic Leader Election in Programmable Matter"
 * By Yuval Emek, Shay Kutten, Ron Lavi, and William K. Moses Jr.
 * https://arxiv.org/abs/1905.00580 */

#ifndef AMOEBOTSIM_ALG_LEADERELECTION_DETERMINISTIC_H_
#define AMOEBOTSIM_ALG_LEADERELECTION_DETERMINISTIC_H_

#include <array>
#include <vector>

#include <QString>

#include "core/amoebotparticle.h"
#include "core/amoebotsystem.h"

using namespace std;

class LeaderElectionDeterministicParticle : public AmoebotParticle {
public:
  enum class State {
    None,
    Initlialization,
    Candidate,
    ForestFormation,
    Leader
  };

  State state;

  // Denotes for each boundary whether this particle is the head of a segment
  std::vector<bool> segHeads = {};

  // Denotes for each boundary the label of this particle
  std::vector<int> labels = {};

  // Denotes for each boundary the count of this particle's segment, if it is
  // the head of this segment
  std::vector<int> counts = {};

  std::vector<int> successors = {};

  std::vector<int> predecessors = {};

  // Denotes for each boundary whether this particle's segment has requested
  // the count of the next segment, if this particle is the head of this segment
  std::vector<bool> countsRequested = {};

  // Denotes for each boundary whether this particle's segment has requested
  // a merge with the next segment, if this particle is the head of this segment
  std::vector<bool> mergesRequested = {};

  // Denotes for each boundary whether this particle's segment has initiated
  // lexicographic comparison with the next segment, 
  // if this particle is the head of this segment
  std::vector<bool> lexicoGraphicComparisons = {};

  std::vector<bool> sentLabels = {};

  std::vector<bool> sentNbrLabels = {};

  std::vector<bool> reqLabels = {};

  std::vector<bool> reqLabelsForNbr = {};

  std::vector<bool> reqNbrLabels = {};

  std::vector<bool> receivedLabels = {};

  std::vector<bool> receivedNbrLabels = {};

  std::vector<int> internalLabels = {};

  std::vector<int> nbrLabels = {};

  std::vector<bool> endOfSegments = {};

  std::vector<bool> endOfNbrSegments = {};

  std::vector<int> firstLargerLabels = {};

  std::vector<bool> terminationDetections = {};

  // Constructs a new particle with a node position for its head, a global
  // compass direction from its head to its tail (-1 if contracted), an offset
  // for its local compass, and a system which it belongs to.
  LeaderElectionDeterministicParticle(const Node head, const int globalTailDir,
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
  LeaderElectionDeterministicParticle &nbrAtLabel(int label) const;

  // Returns the string to be displayed when this particle is inspected; used
  // to snapshot the current values of this particle's memory at runtime.
  virtual QString inspectionText() const;

  // Returns true iff the calling particle lies on the outer boundary
  // of the particle system
  bool isBoundaryParticle();

  // Returns the number of boundaries on which the calling particle lies
  int numBoundaries();

  // Initialize the vector of labels with the appropriate label for each boundary.
  // -1 for bend left
  //  0 for straight, 
  //  1 for bend right, 
  //  2 for sharp bend right
  //  3 for backwards
  void setLabels();

  // Returns the direction of the next (resp. previous) particle on the given boundary.
  int nextDir(int boundary);
  int prevDir(int boundary);

  // Resets lexicographic comparison variables and passes cleanup token throughout the segment.
  void cleanup(int boundary);
  void cleanupForNbr(int boundary);

protected:
  // The LeaderElectionToken struct provides a general framework of any token
  // under the General Leader Election algorithm.
  struct LeaderElectionToken : public Token {
    // origin is used to define the direction (label) that a LeaderElectionToken
    // was received from.
    int origin;
  };

  // Used to request the count from the next segment
  struct CountRequestToken : public LeaderElectionToken {
    CountRequestToken(int origin = -1) {
      this->origin = origin;
    }
  };
  // Used to send the count to the previous segment
  struct CountToken : public LeaderElectionToken {
    int count;
    CountToken(int origin = -1, int count = 0) {
      this->origin = origin;
      this->count = count;
    }
  };
  // Used to request a merge with the next segment
  struct MergeRequestCountToken : public LeaderElectionToken {
    int count;
    MergeRequestCountToken(int origin = -1, int count = 0) {
      this->origin = origin;
      this->count = count;
    }
  };
  // Used to acknowledge a merge with the previous segment
  struct MergeAckToken : public LeaderElectionToken {
    int count;
    MergeAckToken(int origin = -1, int count = 0) {
      this->origin = origin;
      this->count = count;
    }
  };
  // Used to decline a merge with the previous segment
  struct MergeNackToken : public LeaderElectionToken {
    MergeNackToken(int origin = -1) {
      this->origin = origin;
    }
  };

  /*
   * Lexicographic comparison tokens
   * 
   * LexCompRequestNbrLabelToken : request a label from the next segment
   * LexCompReturnNbrLabelToken : send label back to the previous segment
   * LexCompReturnNbrEndOfSegmentToken : signal end of segment to previous segment
   * LexCompReqLabelToken : request internal label for comparison to next segment
   * LexCompReqLabelForNbrToken : request internal label for previous segment
   * LexCompReturnLabelToken : return internal label for comparison to next segment
   * LexCompEndOfSegmentToken : signal end of segment
   * LexCompReturnLabelForNbrToken : return internal label for previous segment
   * LexCompEndOfSegmentForNbrToken : signal end of segment to the previous segment
   * LexCompInterruptNextToken : sent to the next segment to interrupt lexicographic comparison
   * LexCompInterruptPrevToken : sent to the previous segment to interrupt lexicographic comparison
   * LexCompMergeRequestToken : used to request a merge after lexicographic comparison
   * LexCompCleanupToken : used to reset variables throughout the segment
   * LexCompCleanupForNbrToken : used to reset variables throughout the segment
   */
  struct LexCompRequestNbrLabelToken : public LeaderElectionToken {
    LexCompRequestNbrLabelToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompReturnNbrLabelToken : public LeaderElectionToken {
    int label;
    LexCompReturnNbrLabelToken(int origin = -1, int label = 0) {
      this->origin = origin;
      this->label = label;
    }
  };
  struct LexCompReturnNbrEndOfSegmentToken : public LeaderElectionToken {
    LexCompReturnNbrEndOfSegmentToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompReqLabelToken : public LeaderElectionToken {
    LexCompReqLabelToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompReqLabelForNbrToken : public LeaderElectionToken {
    LexCompReqLabelForNbrToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompReturnLabelToken : public LeaderElectionToken {
    int label;
    LexCompReturnLabelToken(int origin = -1, int label = 0) {
      this->origin = origin;
      this->label = label;
    }
  };
  struct LexCompEndOfSegmentToken : public LeaderElectionToken {
    LexCompEndOfSegmentToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompReturnLabelForNbrToken : public LeaderElectionToken {
    int label;
    LexCompReturnLabelForNbrToken(int origin = -1, int label = 0) {
      this->origin = origin;
      this->label = label;
    }
  };
  struct LexCompEndOfSegmentForNbrToken : public LeaderElectionToken {
    LexCompEndOfSegmentForNbrToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompInterruptNextToken : public LeaderElectionToken {
    LexCompInterruptNextToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompInterruptPrevToken : public LeaderElectionToken {
    LexCompInterruptPrevToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompMergeRequestToken : public LeaderElectionToken {
    int count;
    LexCompMergeRequestToken(int origin = -1, int count = 0) {
      this->origin = origin;
      this->count = count;
    }
  };
  struct LexCompCleanupToken : public LeaderElectionToken {
    LexCompCleanupToken(int origin = -1) {
      this->origin = origin;
    }
  };
  struct LexCompCleanupForNbrToken : public LeaderElectionToken {
    LexCompCleanupForNbrToken(int origin = -1) {
      this->origin = origin;
    }
  };

  /*
   * Termination detection tokens
   * TerminationDetectionToken : sent in the counter-clockwise direction to detect termination
   * TerminationDetectionReturnToken : sent back in clockwise direction with result
   * TerminationToken : signals to other segments that termination has been detected
   */
  struct TerminationDetectionToken : public LeaderElectionToken {
    int count;
    int ttl;
    int traversed;
    TerminationDetectionToken(int origin = -1, int count = 0, int ttl = 0, int traversed = 0) {
      this->origin = origin;
      this->count = count;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
  struct TerminationDetectionReturnToken : public LeaderElectionToken {
    int count;
    int ttl;
    int traversed;
    bool termination;
    TerminationDetectionReturnToken(int origin = -1, int count = 0, int ttl = 0, int traversed = 0, bool termination = false) {
      this->origin = origin;
      this->count = count;
      this->ttl = ttl;
      this->traversed = traversed;
      this->termination = termination;
    }
  };
  struct TerminationToken : public LeaderElectionToken {
    int ttl;
    int traversed;
    TerminationToken(int origin = -1, int ttl = 0, int traversed = 0) {
      this->origin = origin;
      this->ttl = ttl;
      this->traversed = traversed;
    }
  };
};

class LeaderElectionDeterministicSystem : public AmoebotSystem {
public:
  // Constructs a system of LeaderElectionDeterministicParticles with an optionally
  // specified size (#particles).
  LeaderElectionDeterministicSystem(int numParticles = 100);

  // Checks whether or not the system's run of the Leader Election algorithm has
  // terminated (all particles in state Finished or Leader).
  bool hasTerminated() const override;
};
#endif // AMOEBOTSIM_ALG_LEADERELECTION_DETERMINISTIC_H_