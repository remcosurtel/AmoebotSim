/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Stationary and deterministic leader election.
 * Based on the paper "Stationary and Deterministic Leader Election in 
 * Self-organizing Particle Systems"
 * By Rida A. Bazzi and Joseph L. Briones.
 * DOI: 10.1007/978-3-030-34992-9_3 */

#include "alg/leaderelection_stationary_deterministic.h"

#include <set>

#include <QtGlobal>

#include <QDebug>

using namespace std;

//----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionStationaryDeterministicParticle::LeaderElectionStationaryDeterministicParticle(
    const Node head, const int globalTailDir, const int orientation,
    AmoebotSystem &system, State state)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state) {
  borderColorLabels.fill(-1);
  borderPointColorLabels.fill(-1);
  borderPointBetweenEdgeColorLabels.fill(-1);
}

void LeaderElectionStationaryDeterministicParticle::activate() {
  qDebug() << "Particle: " + QString::number(head.x) + "," + QString::number(head.y);
  if (state == State::IdentificationLabeling) {
    // Determine the number of neighbors of the current particle.
    // If there are no neighbors, then that means the particle is the only
    // one in the system and should declare itself as the leader.
    // If it is surrounded by 6 neighbors, then it cannot participate in
    // leader election.
    // Otherwise, the particle may participate in leader election and must
    // generate nodes to do so.
    int numNbrs = getNumberOfNbrs();
    if (numNbrs == 0) {
      state = State::Leader;
      return;
    }
    else if (numNbrs == 6) {
      state = State::Finished;
    }
    else {
      // Initialize 6 nodes
      qDebug() << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Initializing nodes... @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
      for (int dir = 0; dir < 6; dir++) {
        if (!hasNbrAtLabel((dir + 1) % 6) || !hasNbrAtLabel(dir)) {
          LeaderElectionNode* node = new LeaderElectionNode();
          node->particle = this;
          node->nodeDir = dir;
          node->nextNodeDir = -((dir + 5) % 6) - 1;
          node->prevNodeDir = -((dir + 1) % 6) - 1;
          node->nodeState = State::StretchExpansion;

          if (hasNbrAtLabel((dir + 1) % 6) || hasNbrAtLabel(dir)) {
            if (hasNbrAtLabel((dir + 1) % 6)) {
              node->prevNodeDir = (dir + 1) % 6;
              node->prevNodeClone = true;
            }
            else {
              node->nextNodeDir = dir;
              node->nextNodeClone = true;
            }
            // Node is shared between two particles -> label -1
            node->unaryLabel = -1;
            node->count = -1;
            node->paintNode(0xff0000); // Red
          }
          else {
            // Node belongs only to this particle -> label +1
            node->unaryLabel = 1;
            node->count = 1;
            node->paintNode(0x00bdff); // Cyan
          }
          nodes.push_back(node);
        }
      }
      if (nodes.size() > 0) {
        state = State::StretchExpansion;
        return;
      }
      else {
        state = State::Demoted;
        return;
      }
    }
  }
  else if (state == State::StretchExpansion) {
    if (nodes.size() == 0) {
      qDebug() << "No nodes; finished.";
      state = State::Finished;
      return;
    }
    // Wait for all neighbors to reach stretch expansion state.
    for (int dir = 0; dir < 6; dir++) {
      if (hasNbrAtLabel(dir)) {
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
        if (nbr.state == State::IdentificationLabeling) {
          return;
        }
      }
    }
    // For all nodes: if clone, synchronize before activating
    for (LeaderElectionNode* node : nodes) {
      if (node->nextNodeClone) {
        LeaderElectionNode* clone = node->nextNode();
        if (clone->cloneChange) {
          node->count = clone->count;
          node->countSent = clone->countSent;
          node->mergeAck = clone->mergeAck;
          node->mergeDir = clone->mergeDir;
          node->mergePending = clone->mergePending;
          node->nodeState = clone->nodeState;
          node->predecessor = clone->predecessor;
          node->subPhase = clone->subPhase;
          node->successor = clone->successor;

          node->lexCompInit = clone->lexCompInit;
          node->lexicographicComparisonLeft = clone->lexicographicComparisonLeft;
          node->lexicographicComparisonRight = clone->lexicographicComparisonRight;
          node->requestedNbrLabel = clone->requestedNbrLabel;
          node->receivedNbrLabel = clone->receivedNbrLabel;
          node->NbrLabel = clone->NbrLabel;
          node->requestedLabel = clone->requestedLabel;
          node->receivedLabel = clone->receivedLabel;
          node->internalLabel = clone->internalLabel;
          node->firstLargerLabel = clone->firstLargerLabel;
          node->retrieved = clone->retrieved;
          node->requestedLabelForNbr = clone->requestedLabelForNbr;
          node->receivedLabelForNbr = clone->receivedLabelForNbr;
          node->internalLabelForNbr = clone->internalLabelForNbr;
          node->retrievedForNbr = clone->retrievedForNbr;
          node->receivedLabelRequestFromNbr = clone->receivedLabelRequestFromNbr;

          clone->cloneChange = false;
        }
        node->cloneChange = true;
      }
      else if (node->prevNodeClone) {
        LeaderElectionNode* clone = node->prevNode();
        if (clone->cloneChange) {
          node->count = clone->count;
          node->countSent = clone->countSent;
          node->mergeAck = clone->mergeAck;
          node->mergeDir = clone->mergeDir;
          node->mergePending = clone->mergePending;
          node->nodeState = clone->nodeState;
          node->predecessor = clone->predecessor;
          node->subPhase = clone->subPhase;
          node->successor = clone->successor;

          node->lexCompInit = clone->lexCompInit;
          node->lexicographicComparisonLeft = clone->lexicographicComparisonLeft;
          node->lexicographicComparisonRight = clone->lexicographicComparisonRight;
          node->requestedNbrLabel = clone->requestedNbrLabel;
          node->receivedNbrLabel = clone->receivedNbrLabel;
          node->NbrLabel = clone->NbrLabel;
          node->requestedLabel = clone->requestedLabel;
          node->receivedLabel = clone->receivedLabel;
          node->internalLabel = clone->internalLabel;
          node->firstLargerLabel = clone->firstLargerLabel;
          node->retrieved = clone->retrieved;
          node->requestedLabelForNbr = clone->requestedLabelForNbr;
          node->receivedLabelForNbr = clone->receivedLabelForNbr;
          node->internalLabelForNbr = clone->internalLabelForNbr;
          node->retrievedForNbr = clone->retrievedForNbr;
          node->receivedLabelRequestFromNbr = clone->receivedLabelRequestFromNbr;

          clone->cloneChange = false;
        }
        node->cloneChange = true;
      }
    }
    // Activate each node. Nodes run stretch expansion.
    qDebug() << "===================== Activating nodes... =====================";
    for (int i = 0; i < nodes.size(); i++){
      nodes.at(i)->activate();
    }
    qDebug() << "Painting nodes...";
    for (int i = 0; i < nodes.size(); i++){
      LeaderElectionNode* node = nodes.at(i);
      if (node->mergePending) {
        node->paintNode(0xb900ff); // Purple
      }
      else if (node->lexicographicComparisonRight) {
        node->paintNode(0x00ff00); // Green
      }
      else if (node->predecessor == nullptr) {
        node->paintNode(0xff9b00); // Gold
      }
      else {
        node->paintNode(0x000000); // Black
      }
    }
  }
  return;
}

int LeaderElectionStationaryDeterministicParticle::headMarkDir() const {
  return -1;
  // return globalToLocalDir(0);
}

int LeaderElectionStationaryDeterministicParticle::headMarkColor() const {
  return -1;
  // return 0x000000;
}

LeaderElectionStationaryDeterministicParticle &
LeaderElectionStationaryDeterministicParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<LeaderElectionStationaryDeterministicParticle>(label);
}

QString LeaderElectionStationaryDeterministicParticle::inspectionText() const {
  QString text;
  QString indent = "    ";

  text = "";
  text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
          ")\n";
  text += "orientation: " + QString::number(orientation) + "\n";
  text += "state: ";
  text += [this]() {
    switch (state) {
    case State::IdentificationLabeling:
      return "IdentificationLabeling";
    case State::StretchExpansion:
      return "StretchExpansion";
    case State::Demoted:
      return "Demoted";
    case State::Candidate:
      return "Candidate";
    case State::Finished:
      return "Finished";
    case State::Leader:
      return "Leader";
    default:
      return "no state";
    }
  }();
  text += "\n";
  text += "has leader election tokens: " +
          QString::number(countTokens<LeaderElectionToken>()) + "\n";
  text += "has lexicographic comparison tokens: " +
          QString::number(countTokens<LexCompToken>()) + "\n";
  text += "\n\n";

  for (int i = 0; i < nodes.size(); i++) {
    LeaderElectionNode* node = nodes.at(i);
    text += "Node, dir: " + QString::number(i) + ", " + QString::number(node->nodeDir) + "\n";
    text += "Global dir: " + QString::number(localToGlobalDir(node->nodeDir)) + "\n";
    if (node->nextNodeClone) {
      text += "Clone: next\n";
    }
    else if (node->prevNodeClone) {
      text += "Clone: prev\n";
    }
    else {
      text += "Clone: N/A\n";
    }
    text += "Next, prev node dir: " + QString::number(node->nextNodeDir) + ", " + QString::number(node->prevNodeDir) + "\n";
    text += "Unary label: " + QString::number(node->unaryLabel) + "\n";
    text += "Head: ";
    if (node->predecessor == nullptr) {
      text += "true\n";
    }
    else {
      text += "false\n";
    }
    text += "Tail: ";
    if (node->successor == nullptr) {
      text += "true\n";
    }
    else {
      text += "false\n";
    }
    text += "Count: " + QString::number(node->count) + "\n";
    if (node->mergePending) {
      text += "Merge pending: true\n";
    }
    else {
      text += "Merge pending: false\n";
    }
    if (node->predecessor == nullptr) {
      if (node->lexicographicComparisonLeft && node->lexicographicComparisonRight){
        text += "Lexicographic comparison: left & right\n";
      }
      else if (node->lexicographicComparisonRight) {
        text += "Lexicographic comparison: right\n";
      }
      else if (node->lexicographicComparisonLeft) {
        text += "Lexicographic comparison: left\n";
      }
      else {
        text += "Lexicographic comparison: false\n";
      }
    }
    text += "\n";
  }

  return text;
}

std::array<int, 18> LeaderElectionStationaryDeterministicParticle::borderColors() const {
  return borderColorLabels;
}

std::array<int, 6> LeaderElectionStationaryDeterministicParticle::borderPointColors() const {
  return borderPointColorLabels;
}

std::array<int, 6> LeaderElectionStationaryDeterministicParticle::borderPointBetweenEdgeColors() const {
  return borderPointBetweenEdgeColorLabels;
}

int LeaderElectionStationaryDeterministicParticle::getNumberOfNbrs() const {
  int count = 0;
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      count++;
    }
  }
  return count;
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN AGENT CODE----------------------------

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::LeaderElectionNode() :
  nodeDir(-1),
  nextNodeDir(-1),
  prevNodeDir(-1),
  particle(nullptr) {}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::activate() {
  if (nodeState == State::StretchExpansion) {
    qDebug() << "Running StretchExpansion...";
    if (predecessor == nullptr) {
      // This node is the head of a stretch
      qDebug() << "Head node...";
      // Handle lexicographic comparison tokens
      // If ack token received -> start lexicographic comparison
      if (hasNodeToken<LexCompAckToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Processing ack token...";
        takeNodeToken<LexCompAckToken>(nextNode()->prevNodeDir);
        if (lexCompInit && !lexicographicComparisonRight) {
          lexicographicComparisonRight = true;
        }
      }
      // If nack token received -> abort lexicographic comparison
      if (hasNodeToken<LexCompNackToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Processing nack token...";
        takeNodeToken<LexCompNackToken>(nextNode()->prevNodeDir);
        if (lexCompInit && !lexicographicComparisonRight) {
          lexCompInit = false;
        }
      }
      // If init token received, send ack or nack token
      if (hasNodeToken<LexCompInitToken>(prevNode()->nextNodeDir)) {
        qDebug() << "Processing init token...";
        std::shared_ptr<LexCompInitToken> token = takeNodeToken<LexCompInitToken>(prevNode()->nextNodeDir);
        int value = token->value;
        if (value == count && !lexicographicComparisonLeft) {
          passNodeToken<LexCompAckToken>(prevNodeDir, std::make_shared<LexCompAckToken>());
          lexicographicComparisonLeft = true;
        }
        else {
          passNodeToken<LexCompNackToken>(prevNodeDir, std::make_shared<LexCompNackToken>());
        }
      }
      // If interrupt token received, cancel lexicographic comparison
      if (hasNodeToken<LexCompInterruptToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Processing interrupt token...";
        takeNodeToken<LexCompInterruptToken>(nextNode()->prevNodeDir);
        if (lexicographicComparisonRight) {
          firstLargerLabel = 0;
          NbrLabel = 0;
          internalLabel = 0;
          requestedNbrLabel = false;
          receivedNbrLabel = false;
          requestedLabel = false;
          receivedLabel = false;
          lexCompInit = false;
          lexicographicComparisonRight = false;
          retrieved = false;
          if (successor != nullptr) {
            passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
          }
          countSent = false;
        }
      }
      if (hasNodeToken<LexCompInterruptToken>(prevNode()->nextNodeDir)) {
        qDebug() << "Processing interrupt token...";
        takeNodeToken<LexCompInterruptToken>(prevNode()->nextNodeDir);
        if (lexicographicComparisonLeft) {
          internalLabelForNbr = 0;
          requestedLabelForNbr = false;
          receivedLabelForNbr = false;
          lexicographicComparisonLeft = false;
          retrievedForNbr = false;
          if (successor != nullptr) {
            passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
          }
        }
      }
      
      // =================================== Lexicographic comparison to clockwise adjacent stretch ============================================
      if (lexicographicComparisonRight) {
        // Request labels from the adjacent stretch
        if (!requestedNbrLabel) {
          qDebug() << "Requesting label from adjacent stretch...";
          passNodeToken<LexCompReqStretchLabelToken>(nextNodeDir, std::make_shared<LexCompReqStretchLabelToken>());
          requestedNbrLabel = true;
        }
        // Receive labels from the adjacent stretch after requesting them
        else if (!receivedNbrLabel) {
          if (hasNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing stretch label token...";
            std::shared_ptr<LexCompReturnStretchLabelToken> token = takeNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir);
            NbrLabel = token->value;
            receivedNbrLabel = true;
          }
          // Receive end of stretch tokens from adjacent stretch
          else if (hasNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing end of stretch token...";
            takeNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir);
            NbrLabel = 0;
            receivedNbrLabel = true;
          }
        }
        // Request internal labels
        if (!requestedLabel) {
          if (!retrieved) {
            requestedLabel = true;
            receivedLabel = true;
            internalLabel = unaryLabel;
            retrieved = true;
          }
          else {
            qDebug() << "Requesting internal label...";
            passNodeToken<LexCompRetrieveNextLabelToken>(nextNodeDir, std::make_shared<LexCompRetrieveNextLabelToken>());
            requestedLabel = true;
          }
        }
        // Receive internal labels after requesting them
        else if (!receivedLabel) {
          if (hasNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing internal label token...";
            std::shared_ptr<LexCompNextLabelToken> token = takeNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir);
            internalLabel = token->value;
            receivedLabel = true;
          }
          // Receive end of stretch tokens
          else if (hasNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing internal end of stretch token...";
            takeNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir);
            internalLabel = 0;
            receivedLabel = true;
          }
        }
        // If both labels received, compare them
        if (receivedNbrLabel && receivedLabel) {
          qDebug() << "Comparing received labels...";
          // If all labels thus far have been the same
          if (firstLargerLabel == 0) {
            // Then if the labels are different, remember which was larger
            if (internalLabel > NbrLabel) {
              firstLargerLabel = 1;
            }
            else if (internalLabel < NbrLabel) {
              firstLargerLabel = -1;
            }
          }
          // If one of the labels is 0, then the stretch has exhausted all its labels
          // In this case, the other stretch is lexicographically larger
          if (internalLabel == 0 && NbrLabel != 0) {
            // Adjacent stretch is lexicographically larger -> no merge
            passNodeToken<LexCompInterruptToken>(nextNodeDir, std::make_shared<LexCompInterruptToken>());
            // Clean up
            firstLargerLabel = 0;
            NbrLabel = 0;
            internalLabel = 0;
            requestedNbrLabel = false;
            receivedNbrLabel = false;
            requestedLabel = false;
            receivedLabel = false;
            lexCompInit = false;
            lexicographicComparisonRight = false;
            retrieved = false;
            if (successor != nullptr) {
              passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
            }
            countSent = false;
          }
          else if (internalLabel != 0 && NbrLabel == 0) {
            // This stretch is lexicographically larger -> merge
            // Clean up
            firstLargerLabel = 0;
            NbrLabel = 0;
            internalLabel = 0;
            requestedNbrLabel = false;
            receivedNbrLabel = false;
            requestedLabel = false;
            receivedLabel = false;
            lexCompInit = false;
            lexicographicComparisonRight = false;
            retrieved = false;
            if (successor != nullptr) {
              passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
            }
            countSent = false;
            // Merge
            passNodeToken<LexCompAttemptMergeToken>(nextNodeDir, std::make_shared<LexCompAttemptMergeToken>(-1, count));
            mergePending = true;
            mergeAck = false;
            mergeDir = 1;
          }
          // If both labels are 0, then both strings were of the same length
          if (internalLabel == 0 && NbrLabel == 0) {
            // In this case, the string which had the first larger label is lexicographically larger
            if (firstLargerLabel == -1) {
              // Adjacent stretch is lexicographically larger -> no merge
              // Clean up
              firstLargerLabel = 0;
              NbrLabel = 0;
              internalLabel = 0;
              requestedNbrLabel = false;
              receivedNbrLabel = false;
              requestedLabel = false;
              receivedLabel = false;
              lexCompInit = false;
              lexicographicComparisonRight = false;
              retrieved = false;
              if (successor != nullptr) {
                passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
              }
              countSent = false;
            }
            else if (firstLargerLabel == 1) {
              // This stretch is lexicographically larger -> merge
              // Clean up
              firstLargerLabel = 0;
              NbrLabel = 0;
              internalLabel = 0;
              requestedNbrLabel = false;
              receivedNbrLabel = false;
              requestedLabel = false;
              receivedLabel = false;
              lexCompInit = false;
              lexicographicComparisonRight = false;
              retrieved = false;
              if (successor != nullptr) {
                passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
              }
              countSent = false;
              // Merge
              passNodeToken<LexCompAttemptMergeToken>(nextNodeDir, std::make_shared<LexCompAttemptMergeToken>(-1, count));
              mergePending = true;
              mergeAck = false;
              mergeDir = 1;
            }
            // If both strings were identical, then trigger termination detection
            else {
              // TODO: termination detection
              // Clean up
              firstLargerLabel = 0;
              NbrLabel = 0;
              internalLabel = 0;
              requestedNbrLabel = false;
              receivedNbrLabel = false;
              requestedLabel = false;
              receivedLabel = false;
              lexCompInit = false;
              lexicographicComparisonRight = false;
              retrieved = false;
              if (successor != nullptr) {
                passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
              }
              countSent = false;
            }
          }
          // Reset variables to request next labels
          requestedNbrLabel = false;
          receivedNbrLabel = false;
          requestedLabel = false;
          receivedLabel = false;
        }
      }
      // ========================================== Lexicographic comparison to counter-clockwise adjacent stretch
      if (lexicographicComparisonLeft) {
        // Receive requests for labels
        if (!receivedLabelRequestFromNbr) {
          if (hasNodeToken<LexCompReqStretchLabelToken>(prevNode()->nextNodeDir)) {
            qDebug() << "Processing label request token...";
            takeNodeToken<LexCompReqStretchLabelToken>(prevNode()->nextNodeDir);
            receivedLabelRequestFromNbr = true;
          }
        }
        // Request internal labels
        if (!requestedLabelForNbr) {
          if (!retrievedForNbr) {
            requestedLabelForNbr = true;
            receivedLabelForNbr = true;
            internalLabelForNbr = unaryLabel;
            retrievedForNbr = true;
          }
          else {
            if (successor != nullptr) {
              qDebug() << "Requesting internal label...";
              passNodeToken<LexCompRetrieveNextLabelForNbrToken>(nextNodeDir, std::make_shared<LexCompRetrieveNextLabelForNbrToken>());
              requestedLabelForNbr = true;
            }
            else {
              // This node is the head AND tail of its stretch. Thus it has exhausted all labels
              internalLabelForNbr = 0;
              requestedLabelForNbr = true;
              receivedLabelForNbr = true;
            }
          }
        }
        // Receive internal labels after requesting them
        else if (!receivedLabelForNbr) {
          if (hasNodeToken<LexCompNextLabelForNbrToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing internal label token...";
            std::shared_ptr<LexCompNextLabelForNbrToken> token = takeNodeToken<LexCompNextLabelForNbrToken>(nextNode()->prevNodeDir);
            internalLabelForNbr = token->value;
            receivedLabelForNbr = true;
          }
          // Receive end of stretch tokens
          else if (hasNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir)) {
            qDebug() << "Processing internal end of stretch token...";
            takeNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir);
            internalLabelForNbr = 0;
            receivedLabelForNbr = true;
          }
        }
        // Respond to label request when ready
        if (receivedLabelRequestFromNbr && receivedLabelForNbr) {
          qDebug() << "Sending received internal label...";
          // If there was an internal label, send it
          if (internalLabelForNbr != 0) {
            passNodeToken<LexCompReturnStretchLabelToken>(prevNodeDir, std::make_shared<LexCompReturnStretchLabelToken>(-1, internalLabel));
            receivedLabelRequestFromNbr = false;
            requestedLabelForNbr = false;
            receivedLabelForNbr = false;
            internalLabelForNbr = 0;
          }
          // Otherwise signal end of stretch
          else {
            passNodeToken<LexCompEndOfNbrStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfNbrStretchToken>());
            // Clean up
            internalLabelForNbr = 0;
            receivedLabelRequestFromNbr = false;
            requestedLabelForNbr = false;
            receivedLabelForNbr = false;
            lexicographicComparisonLeft = false;
            retrievedForNbr = false;
            if (successor != nullptr) {
              passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
            }
          }
        }
      }

      // Handle merge tokens
      if (hasNodeToken<MergeNackToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<MergeNackToken>(nextNode()->prevNodeDir);
        mergePending = false;
      }
      if (hasNodeToken<MergeAckToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<MergeAckToken>(nextNode()->prevNodeDir);
        mergeAck = true;
        // Interrupt lexicographic comparison if applicable
        if (lexicographicComparisonLeft) {
          passNodeToken<LexCompInterruptToken>(prevNodeDir, std::make_shared<LexCompInterruptToken>());
          internalLabelForNbr = 0;
          receivedLabelRequestFromNbr = false;
          requestedLabelForNbr = false;
          receivedLabelForNbr = false;
          lexicographicComparisonLeft = false;
          retrievedForNbr = false;
          if (successor != nullptr) {
            passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
          }
        }
      }
      if (hasNodeToken<MergeRequestToken>(prevNode()->nextNodeDir)) {
        qDebug() << "Merge request received.";
        if (mergePending) {
          qDebug() << "Merge pending.";
          takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
          qDebug() << "Request declined.";
        }
        else {
          qDebug() << "No merge pending.";
          takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
          passNodeToken<MergeAckToken>(prevNodeDir, std::make_shared<MergeAckToken>());
          mergePending = true;
          mergeAck = true;
          mergeDir = -1;
          qDebug() << "Request acknowledged.";
          // Interrupt lexicographic comparison if applicable
          if (lexCompInit) {
            passNodeToken<LexCompInterruptToken>(nextNodeDir, std::make_shared<LexCompInterruptToken>());
          }
          firstLargerLabel = 0;
          NbrLabel = 0;
          internalLabel = 0;
          requestedNbrLabel = false;
          receivedNbrLabel = false;
          requestedLabel = false;
          receivedLabel = false;
          lexCompInit = false;
          lexicographicComparisonRight = false;
          retrieved = false;
          if (successor != nullptr) {
            passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
          }
          countSent = false;
        }
      }
      if (successor == nullptr) {
        // This node is also the tail of a stretch,
        // therefore it is a stretch of 1 node
        qDebug() << "Tail node...";
        if (unaryLabel > 0 && !mergePending) {
          LeaderElectionNode* next = nextNode(true);
          if (unaryLabel > next->count && unaryLabel + next->count <= 6) {
            // Send a merge request
            qDebug() << "Sending merge request...";
            passNodeToken<MergeRequestToken>(nextNodeDir, std::make_shared<MergeRequestToken>());
            mergePending = true;
            mergeAck = false;
            mergeDir = 1;
            qDebug() << "Merge request sent.";
            // Interrupt lexicographic comparison if applicable
            if (lexicographicComparisonLeft) {
              passNodeToken<LexCompInterruptToken>(prevNodeDir, std::make_shared<LexCompInterruptToken>());
              // Clean up
              lexicographicComparisonLeft = false;
              retrievedForNbr = false;
              requestedLabelForNbr = false;
              receivedLabelForNbr = false;
              internalLabelForNbr = 0;
              receivedLabelRequestFromNbr = false;
            }
          }
          else if (unaryLabel == next->count && unaryLabel + next->count <= 6) {
            // Same counts -> lexicographic comparison
            // Either next stretch is larger (do nothing)
            // or also of size 1 with label 1 (trigger termination detection)
            if (next->successor == nullptr) {
              // TODO: termination detection

            }
          }
        }
        if (mergePending && mergeAck) {
          qDebug() << "Merging...";
          if (mergeDir == 1) {
            qDebug() << "Merging right...";
            successor = nextNode(true);
            count += successor->count;
          }
          else {
            qDebug() << "Merging left...";
            predecessor = prevNode(true);
          }
          mergePending = false;
          mergeAck = false;
          qDebug() << "Merged.";
        }
      }
      else {
        // Node is head but not tail of stretch
        qDebug() << "Head but not tail node";
        if (hasNodeToken<MergeRequestToken>(prevNode()->nextNodeDir)) {
          qDebug() << "Received merge request";
          if (!mergePending) {
            qDebug() << "No merge pending";
            takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
            predecessor = prevNode(true);
            passNodeToken<MergeAckToken>(prevNodeDir, std::make_shared<MergeAckToken>());
            qDebug() << "Merge request acknowledged";
            // Interrupt lexicographic comparison if applicable
            if (lexCompInit) {
              passNodeToken<LexCompInterruptToken>(nextNodeDir, std::make_shared<LexCompInterruptToken>());
            }
            firstLargerLabel = 0;
            NbrLabel = 0;
            internalLabel = 0;
            requestedNbrLabel = false;
            receivedNbrLabel = false;
            requestedLabel = false;
            receivedLabel = false;
            lexCompInit = false;
            lexicographicComparisonRight = false;
            retrieved = false;
            if (successor != nullptr) {
              passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
            }
            countSent = false;
          }
          else {
            qDebug() << "Merge pending";
            takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
            passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
            qDebug() << "Merge request declined";
          }
        }
        if (mergePending && mergeAck && mergeDir == -1) {
          qDebug() << "Merging left...";
          predecessor = prevNode(true);
          mergePending = false;
          mergeAck = false;
          qDebug() << "Merged.";
        }
        else if (!mergePending) {
          qDebug() << "No merge pending";
          if (!countSent && count > 0 && !lexCompInit) {
            qDebug() << "Sending count towards tail...";
            passNodeToken<CountToken>(nextNodeDir, std::make_shared<CountToken>(-1, count));
            countSent = true;
            qDebug() << "Count sent";
          }
          else if (!lexCompInit) {
            if (hasNodeToken<CountReturnToken>(successor->prevNodeDir)) {
              qDebug() << "Received count return token";
              std::shared_ptr<CountReturnToken> token = takeNodeToken<CountReturnToken>(successor->prevNodeDir);
              int value = token->value;
              countSent = false;
              qDebug() << "Value: " + QString::number(value);
              if (count > 0 && count > value && count + value <= 6) {
                // Attempt to merge with the adjacent stretch
                qDebug() << "Attempting merge...";
                passNodeToken<AttemptMergeToken>(nextNodeDir, std::make_shared<AttemptMergeToken>(-1, count));
                mergePending = true;
                mergeDir = 1;
                // Interrupt lexicographic comparison if applicable
                if (lexicographicComparisonLeft) {
                  // Lexicographic comparison was initiated by the counter-clockwise adjacent stretch
                  // Send interrupt token in this direction
                  passNodeToken<LexCompInterruptToken>(prevNodeDir, std::make_shared<LexCompInterruptToken>());
                  // Clean up lexicographic comparison variables
                  internalLabelForNbr = 0;
                  requestedLabelForNbr = false;
                  receivedLabelForNbr = false;
                  lexicographicComparisonLeft = false;
                  retrievedForNbr = false;
                  receivedLabelRequestFromNbr = false;
                  if (successor != nullptr) {
                    passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
                  }
                }
              }
              else if (count > 0 && count == value && count + value <= 6) {
                qDebug() << "Counts equal, lexicographic comparison...";
                /*
                 * Lexicographic comparison
                 * 
                 * Send a token to initialize lexicographic comparison to the adjacent stretch
                 * Upon receiving this token, the adjacent stretch's head will send an acknowledgement token
                 * After receiving this acknowledgement token, the head will continue to request labels from the
                 * adjacent stretch, each time comparing to a label of its own stretch. 
                 * Lexicographic comparison starts at the head's label and continues towards the tail.
                 * Each time a node sends its label, it sets a retrieved flag to true.
                 * If a further request is made after the tail's value has been sent, the tail will send
                 * an end of stretch token to signal that all labels have been retrieved.
                 * Lexicographic comparison can be interrupted by merges from either side. In this case,
                 * an interrupt token will be sent to let the other stretch know to stop the process and clean up.
                 * To clean up, either because the process was finished or because it was interrupted, all retrieved
                 * flags are set to false again.
                 * 
                 * The head evaluates lexicographic comparison as follows:
                 * In each comparison, if the values are the same, then continue
                 * When the first different value is encountered, the head remembers whose value was bigger
                 * At the end, the head will know which stretch has more labels, i.e. a longer string.
                 * This stretch is lexicographically larger. When both strings are of the same length,
                 * the stretch which had the first larger label is lexicographically larger.
                 * If all labels were equal -> trigger termination detection.
                 */
                if (!lexCompInit) {
                  passNodeToken<LexCompInitToken>(nextNodeDir, std::make_shared<LexCompInitToken>(-1, count));
                  lexCompInit = true;
                }
              }
            }
          }
        }
        else if (mergePending) {
          qDebug() << "Merge pending";
          if (hasNodeToken<MergeNackToken>(successor->prevNodeDir)) {
            qDebug() << "Received nack token; aborting merge";
            takeNodeToken<MergeNackToken>(successor->prevNodeDir);
            mergePending = false;
          }
          else if (hasNodeToken<MergeCountToken>(successor->prevNodeDir)) {
            qDebug() << "Received merge count token; updating count...";
            std::shared_ptr<MergeCountToken> token = takeNodeToken<MergeCountToken>(successor->prevNodeDir);
            int value = token->value;
            count += value;
            mergePending = false;
          }
        }
      }
    }
    else if (successor == nullptr) {
      // Tail node (and not head node)
      qDebug() << "Tail node";
      // Pass lexicographic comparison tokens
      if (hasNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir)) {
        // Receive cleanup tokens
        takeNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir);
        retrieved = false;
      }
      if (hasNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir)) {
        // Receive cleanup tokens
        takeNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir);
        retrievedForNbr = false;
      }
      if (hasNodeToken<LexCompInitToken>(predecessor->nextNodeDir)) {
        // Pass init tokens towards clockwise adjacent stretch
        std::shared_ptr<LexCompInitToken> token = takeNodeToken<LexCompInitToken>(predecessor->nextNodeDir);
        int value = token->value;
        passNodeToken<LexCompInitToken>(nextNodeDir, std::make_shared<LexCompInitToken>(-1, value));
      }
      if (hasNodeToken<LexCompAckToken>(nextNode()->prevNodeDir)) {
        // Pass ack tokens towards head
        takeNodeToken<LexCompAckToken>(nextNode()->prevNodeDir);
        passNodeToken<LexCompAckToken>(prevNodeDir, std::make_shared<LexCompAckToken>());
      }
      if (hasNodeToken<LexCompNackToken>(nextNode()->prevNodeDir)) {
        // Pass nack tokens towards head
        takeNodeToken<LexCompNackToken>(nextNode()->prevNodeDir);
        passNodeToken<LexCompNackToken>(prevNodeDir, std::make_shared<LexCompNackToken>());
      }
      if (hasNodeToken<LexCompInterruptToken>(nextNode()->prevNodeDir)) {
        // Pass interrupt tokens in counter-clockwise direction
        takeNodeToken<LexCompInterruptToken>(nextNode()->prevNodeDir);
        passNodeToken<LexCompInterruptToken>(prevNodeDir, std::make_shared<LexCompInterruptToken>());
      }
      if (hasNodeToken<LexCompInterruptToken>(predecessor->nextNodeDir)) {
        // Pass interrupt tokens in clockwise direction
        takeNodeToken<LexCompInterruptToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompInterruptToken>(nextNodeDir, std::make_shared<LexCompInterruptToken>());
      }
      if (hasNodeToken<LexCompRetrieveNextLabelToken>(predecessor->nextNodeDir)) {
        // Handle retrieve internal label tokens
        takeNodeToken<LexCompRetrieveNextLabelToken>(predecessor->nextNodeDir);
        // If this node's label hasn't been retrieved yet, send it
        if (!retrieved) {
          passNodeToken<LexCompNextLabelToken>(prevNodeDir, std::make_shared<LexCompNextLabelToken>(-1, unaryLabel));
          retrieved = true;
        }
        // Otherwise send end of stretch token
        else {
          passNodeToken<LexCompEndOfStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfStretchToken>());
        }
      }
      if (hasNodeToken<LexCompRetrieveNextLabelForNbrToken>(predecessor->nextNodeDir)) {
        // Handle retrieve internal label tokens
        takeNodeToken<LexCompRetrieveNextLabelForNbrToken>(predecessor->nextNodeDir);
        // If this node's label hasn't been retrieved yet, send it
        if (!retrievedForNbr) {
          passNodeToken<LexCompNextLabelForNbrToken>(prevNodeDir, std::make_shared<LexCompNextLabelForNbrToken>(-1, unaryLabel));
          retrievedForNbr = true;
        }
        // Otherwise send end of stretch token
        else {
          passNodeToken<LexCompEndOfStretchForNbrToken>(prevNodeDir, std::make_shared<LexCompEndOfStretchForNbrToken>());
        }
      }
      if (hasNodeToken<LexCompReqStretchLabelToken>(predecessor->nextNodeDir)) {
        // Pass label requests towards the adjacent stretch
        takeNodeToken<LexCompReqStretchLabelToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompReqStretchLabelToken>(nextNodeDir, std::make_shared<LexCompReqStretchLabelToken>());
      }
      if (hasNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir)) {
        // Pass labels back towards the head of the stretch
        std::shared_ptr<LexCompReturnStretchLabelToken> token = takeNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir);
        int value = token->value;
        passNodeToken<LexCompReturnStretchLabelToken>(prevNodeDir, std::make_shared<LexCompReturnStretchLabelToken>(-1, value));
      }
      if (hasNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir)) {
        // Pass the adjacent stretch's end of stretch token towards the head
        takeNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir);
        passNodeToken<LexCompEndOfNbrStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfNbrStretchToken>());
      }
      if (hasNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir)) {
        // Receive attempt merge tokens and merge if requirements met
        std::shared_ptr<LexCompAttemptMergeToken> token = takeNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir);
        count = token->value;
        LeaderElectionNode* headNbr = nextNode(true);
        if (count > 0 && count == headNbr->count && count + headNbr->count <= 6) {
          passNodeToken<MergeRequestToken>(nextNodeDir, std::make_shared<MergeRequestToken>());
          mergePending = true;
          mergeDir = 1;
        }
        else {
          // Counts have changed since communication, abort merge.
          qDebug() << "Requirements not satisfied, cancelling merge";
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
          qDebug() << "Nack token sent";
        }
      }

      if (hasNodeToken<CountToken>(predecessor->nextNodeDir)) {
        qDebug() << "Received count token";
        std::shared_ptr<CountToken> token = takeNodeToken<CountToken>(predecessor->nextNodeDir);
        int value = token->value;
        count = value;
        qDebug() << "Value: " + QString::number(value);
        LeaderElectionNode* headNbr = nextNode(true);
        passNodeToken<CountReturnToken>(prevNodeDir, std::make_shared<CountReturnToken>(-1, headNbr->count));
        qDebug() << "Sent return count token";
      }
      if (hasNodeToken<AttemptMergeToken>(predecessor->nextNodeDir)) {
        qDebug() << "Received attempt merge token";
        std::shared_ptr<AttemptMergeToken> token = takeNodeToken<AttemptMergeToken>(predecessor->nextNodeDir);
        int value = token->value;
        count = value;
        qDebug() << "Value: " + QString::number(value);
        LeaderElectionNode* headNbr = nextNode(true);
        if (count > 0 && count > headNbr->count && count + headNbr->count <= 6) {
          qDebug() << "Sending merge request token";
          passNodeToken<MergeRequestToken>(nextNodeDir, std::make_shared<MergeRequestToken>());
          mergePending = true;
          mergeDir = 1;
        }
        else {
          // Counts have changed since communication, abort merge.
          qDebug() << "Requirements not satisfied, cancelling merge";
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
          qDebug() << "Nack token sent";
        }
      }
      if (mergePending) {
        qDebug() << "Merge pending";
        if (hasNodeToken<MergeAckToken>(nextNode()->prevNodeDir)) {
          qDebug() << "Received merge ack token";
          successor = nextNode(true);
          mergePending = false;
          qDebug() << "Merged";
          passNodeToken<MergeCountToken>(prevNodeDir, std::make_shared<MergeCountToken>(-1, successor->count));
          qDebug() << "Merge count token sent";
        }
        else if (hasNodeToken<MergeNackToken>(nextNode()->prevNodeDir)) {
          qDebug() << "Received merge nack token";
          mergePending = false;
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
          qDebug() << "Merge nack token forwarded";
        }
      }
    }
    else if (predecessor != nullptr && successor != nullptr) {
      // Internal node
      qDebug() << "Internal node";
      // Pass lexicographic comparison topkens
      if (hasNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir)) {
        // Pass cleanup tokens towards tail
        takeNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir);
        retrieved = false;
        passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
      }
      if (hasNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir)) {
        takeNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir);
        retrievedForNbr = false;
        passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
      }
      if (hasNodeToken<LexCompInitToken>(predecessor->nextNodeDir)) {
        // Pass init tokens towards clockwise adjacent stretch
        std::shared_ptr<LexCompInitToken> token = takeNodeToken<LexCompInitToken>(predecessor->nextNodeDir);
        int value = token->value;
        passNodeToken<LexCompInitToken>(nextNodeDir, std::make_shared<LexCompInitToken>(-1, value));
      }
      if (hasNodeToken<LexCompAckToken>(successor->prevNodeDir)) {
        // Pass ack tokens towards head
        takeNodeToken<LexCompAckToken>(successor->prevNodeDir);
        passNodeToken<LexCompAckToken>(prevNodeDir, std::make_shared<LexCompAckToken>());
      }
      if (hasNodeToken<LexCompNackToken>(successor->prevNodeDir)) {
        // Pass nack tokens towards head
        takeNodeToken<LexCompNackToken>(successor->prevNodeDir);
        passNodeToken<LexCompNackToken>(prevNodeDir, std::make_shared<LexCompNackToken>());
      }
      if (hasNodeToken<LexCompInterruptToken>(successor->prevNodeDir)) {
        // Pass interrupt tokens in counter-clockwise direction
        takeNodeToken<LexCompInterruptToken>(successor->prevNodeDir);
        passNodeToken<LexCompInterruptToken>(prevNodeDir, std::make_shared<LexCompInterruptToken>());
      }
      if (hasNodeToken<LexCompInterruptToken>(predecessor->nextNodeDir)) {
        // Pass interrupt tokens in clockwise direction
        takeNodeToken<LexCompInterruptToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompInterruptToken>(nextNodeDir, std::make_shared<LexCompInterruptToken>());
      }
      if (hasNodeToken<LexCompRetrieveNextLabelToken>(predecessor->nextNodeDir)) {
        // Pass retrieve internal label tokens
        takeNodeToken<LexCompRetrieveNextLabelToken>(predecessor->nextNodeDir);
        // If this node's label hasn't been retrieved yet, send it
        if (!retrieved) {
          passNodeToken<LexCompNextLabelToken>(prevNodeDir, std::make_shared<LexCompNextLabelToken>(-1, unaryLabel));
          retrieved = true;
        }
        // Otherwise pass the token
        else {
          passNodeToken<LexCompRetrieveNextLabelToken>(nextNodeDir, std::make_shared<LexCompRetrieveNextLabelToken>());
        }
      }
      if (hasNodeToken<LexCompRetrieveNextLabelForNbrToken>(predecessor->nextNodeDir)) {
        // Pass retrieve internal label tokens for counter-clockwise adjacent stretch
        takeNodeToken<LexCompRetrieveNextLabelForNbrToken>(predecessor->nextNodeDir);
        // If this node's label hasn't been retrieved yet, send it
        if (!retrievedForNbr) {
          passNodeToken<LexCompNextLabelForNbrToken>(prevNodeDir, std::make_shared<LexCompNextLabelForNbrToken>(-1, unaryLabel));
          retrievedForNbr = true;
        }
        // Otherwise pass the token
        else {
          passNodeToken<LexCompRetrieveNextLabelForNbrToken>(nextNodeDir, std::make_shared<LexCompRetrieveNextLabelForNbrToken>());
        }
      }
      if (hasNodeToken<LexCompNextLabelToken>(successor->prevNodeDir)) {
        // Pass retrieved internal labels towards the head
        std::shared_ptr<LexCompNextLabelToken> token = takeNodeToken<LexCompNextLabelToken>(successor->prevNodeDir);
        int value = token->value;
        passNodeToken<LexCompNextLabelToken>(prevNodeDir, std::make_shared<LexCompNextLabelToken>(-1, value));
      }
      if (hasNodeToken<LexCompNextLabelForNbrToken>(successor->prevNodeDir)) {
        // Pass retrieved internal labels towards the head
        std::shared_ptr<LexCompNextLabelForNbrToken> token = takeNodeToken<LexCompNextLabelForNbrToken>(successor->prevNodeDir);
        int value = token->value;
        passNodeToken<LexCompNextLabelForNbrToken>(prevNodeDir, std::make_shared<LexCompNextLabelForNbrToken>(-1, value));
      }
      if (hasNodeToken<LexCompEndOfStretchToken>(successor->prevNodeDir)) {
        // Pass internal end of stretch tokens towards the head
        takeNodeToken<LexCompEndOfStretchToken>(successor->prevNodeDir);
        passNodeToken<LexCompEndOfStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfStretchToken>());
      }
      if (hasNodeToken<LexCompEndOfStretchForNbrToken>(successor->prevNodeDir)) {
        // Pass internal end of stretch tokens towards the head
        takeNodeToken<LexCompEndOfStretchForNbrToken>(successor->prevNodeDir);
        passNodeToken<LexCompEndOfStretchForNbrToken>(prevNodeDir, std::make_shared<LexCompEndOfStretchForNbrToken>());
      }
      if (hasNodeToken<LexCompReqStretchLabelToken>(predecessor->nextNodeDir)) {
        // Pass label requests towards the adjacent stretch
        takeNodeToken<LexCompReqStretchLabelToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompReqStretchLabelToken>(nextNodeDir, std::make_shared<LexCompReqStretchLabelToken>());
      }
      if (hasNodeToken<LexCompReturnStretchLabelToken>(successor->prevNodeDir)) {
        // Pass labels back towards the head of the stretch
        std::shared_ptr<LexCompReturnStretchLabelToken> token = takeNodeToken<LexCompReturnStretchLabelToken>(successor->prevNodeDir);
        int value = token->value;
        passNodeToken<LexCompReturnStretchLabelToken>(prevNodeDir, std::make_shared<LexCompReturnStretchLabelToken>(-1, value));
      }
      if (hasNodeToken<LexCompEndOfNbrStretchToken>(successor->prevNodeDir)) {
        // Pass the adjacent stretch's end of stretch token towards the head
        takeNodeToken<LexCompEndOfNbrStretchToken>(successor->prevNodeDir);
        passNodeToken<LexCompEndOfNbrStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfNbrStretchToken>());
      }
      if (hasNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir)) {
        // Pass merge attempt tokens towards tail
        std::shared_ptr<LexCompAttemptMergeToken> token = takeNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir);
        int value = token->value;
        passNodeToken<LexCompAttemptMergeToken>(nextNodeDir, std::make_shared<LexCompAttemptMergeToken>(-1, value));
      }

      // Pass count and merge tokens
      if (hasNodeToken<CountToken>(predecessor->nextNodeDir)) {
        // Pass on count tokens towards the tail
        std::shared_ptr<CountToken> token = takeNodeToken<CountToken>(predecessor->nextNodeDir);
        int value = token->value;
        passNodeToken<CountToken>(nextNodeDir, std::make_shared<CountToken>(-1, value));
      }
      if (hasNodeToken<CountReturnToken>(successor->prevNodeDir)) {
        // Pass on count return tokens towards the head
        std::shared_ptr<CountReturnToken> token = takeNodeToken<CountReturnToken>(successor->prevNodeDir);
        int value = token->value;
        passNodeToken<CountReturnToken>(prevNodeDir, std::make_shared<CountReturnToken>(-1, value));
      }
      if (hasNodeToken<AttemptMergeToken>(predecessor->nextNodeDir)) {
        // Pass on merge attempt tokens towards the tail
        std::shared_ptr<AttemptMergeToken> token = takeNodeToken<AttemptMergeToken>(predecessor->nextNodeDir);
        int value = token->value;
        passNodeToken<AttemptMergeToken>(nextNodeDir, std::make_shared<AttemptMergeToken>(-1, value));
      }
      if (hasNodeToken<MergeNackToken>(successor->prevNodeDir)) {
        // Pass on merge nack tokens towards the head
        takeNodeToken<MergeNackToken>(successor->prevNodeDir);
        passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
      }
      if (hasNodeToken<MergeCountToken>(successor->prevNodeDir)) {
        // Pass on merge count tokens towards the head
        std::shared_ptr<MergeCountToken> token = takeNodeToken<MergeCountToken>(successor->prevNodeDir);
        int value = token->value;
        passNodeToken<MergeCountToken>(prevNodeDir, std::make_shared<MergeCountToken>(-1, value));
      }
    }
  }
}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::paintNode(
    const int color) {
  // paint a node
  // particle->borderColorLabels.at(3 * particle->localToGlobalDir(nodeDir) + 2) = color;
  particle->borderPointBetweenEdgeColorLabels.at(particle->localToGlobalDir(nodeDir)) = color;
}

template <class TokenType>
bool LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
hasNodeToken(int dir, bool checkClone) const{
    if (nextNodeClone && checkClone) {
      LeaderElectionNode* clone = nextNode();
      int cloneDir = dir;
      if (dir == prevNodeDir) {
        cloneDir = clone->prevNodeDir;
      }
      else if (dir == nextNodeDir) {
        cloneDir = clone->nextNodeDir;
      }
      if (clone->hasNodeToken<TokenType>(cloneDir, false)) {
        return true;
      }
    }
    else if (prevNodeClone && checkClone) {
      LeaderElectionNode* clone = prevNode();
      int cloneDir = dir;
      if (dir == prevNodeDir) {
        cloneDir = clone->prevNodeDir;
      }
      else if (dir == nextNodeDir) {
        cloneDir = clone->nextNodeDir;
      }
      if (clone->hasNodeToken<TokenType>(cloneDir, false)) {
        return true;
      }
    }
    auto prop = [dir,this](const std::shared_ptr<TokenType> token) {
      return token->origin == dir && token->destination == nodeDir;
    };
    return particle->hasToken<TokenType>(prop);
}

template <class TokenType>
std::shared_ptr<TokenType>
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
peekNodeToken(int dir) const {
  auto prop = [dir](const std::shared_ptr<TokenType> token) {
    return token->origin == dir;
  };
  return particle->peekAtToken<TokenType>(prop);
}

template <class TokenType>
std::shared_ptr<TokenType>
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
takeNodeToken(int dir, bool checkClone) {
  if (hasNodeToken<TokenType>(dir, false)) {
    auto prop = [dir,this](const std::shared_ptr<TokenType> token) {
      return token->origin == dir && token->destination == nodeDir;
    };
    return particle->takeToken<TokenType>(prop);
  }
  else {
    if (nextNodeClone && checkClone) {
      LeaderElectionNode* clone = nextNode();
      int cloneDir = dir;
      if (dir == prevNodeDir) {
        cloneDir = clone->prevNodeDir;
      }
      else if (dir == nextNodeDir) {
        cloneDir = clone->nextNodeDir;
      }
      return clone->takeNodeToken<TokenType>(cloneDir, false);
    }
    else if (prevNodeClone && checkClone) {
      LeaderElectionNode* clone = prevNode();
      int cloneDir = dir;
      if (dir == prevNodeDir) {
        cloneDir = clone->prevNodeDir;
      }
      else if (dir == nextNodeDir) {
        cloneDir = clone->nextNodeDir;
      }
      return clone->takeNodeToken<TokenType>(cloneDir, false);
    }
    else {
      Q_ASSERT(false);
    }
  }
}

template <class TokenType>
void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
passNodeToken(int dir, std::shared_ptr<TokenType> token, bool checkClone) {
  int dest = dir;
  LeaderElectionStationaryDeterministicParticle* nbr;
  if (dir == nextNodeDir) {
    if (checkClone && nextNodeClone) {
      dest = nextNode(true)->nodeDir;
      nbr = nextNode(true)->particle;
    }
    else {
      dest = nextNode()->nodeDir;
      nbr = nextNode()->particle;
    }
  }
  else if (dir == prevNodeDir) {
    if (checkClone && prevNodeClone) {
      dest = prevNode(true)->nodeDir;
      nbr = prevNode(true)->particle;
    }
    else {
      dest = prevNode()->nodeDir;
      nbr = prevNode()->particle;
    }
  }
  else {
    nbr = &particle->nbrAtLabel(dir);
  }
  int origin = -1;
  if (dir >= 0) {
    if (dir == nextNodeDir) {
      if (checkClone && nextNodeClone) {
        origin = nextNode(true)->prevNode()->nextNodeDir;
      }
      else {
        origin = nextNode()->prevNodeDir;
      }
    }
    else if (dir == prevNodeDir) {
      if (checkClone && prevNodeClone) {
        origin = prevNode(true)->nextNode()->prevNodeDir;
      }
      else {
        origin = prevNode()->nextNodeDir;
      }
    }
    else {
      for (int i = 0; i < 6; i++) {
        if (nbr->hasNbrAtLabel(i)) {
          if (&nbr->nbrAtLabel(i) == particle) {
            origin = i;
            break;
          }
        }
      }
    }
  }
  else {
    origin = dir;
  }
  token->origin = origin;
  token->destination = dest;
  nbr->putToken(token);
}

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode*
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::nextNode(bool recursion) const {
  if (nextNodeDir < 0) {
    for (int i = 0; i < particle->nodes.size(); i++) {
      if (particle->nodes.at(i)->nodeDir == (nodeDir + 5) % 6) {
        return particle->nodes.at(i);
      }
    }
  }
  Q_ASSERT(0 <= nextNodeDir < 6);

  LeaderElectionStationaryDeterministicParticle* nextNbr =
      &particle->nbrAtLabel(nextNodeDir);

  int originLabel = -1;
  for (int i = 0; i < 6; i++) {
    if (nextNbr->hasNbrAtLabel(i)){
      if (&nextNbr->nbrAtLabel(i) == particle) {
        originLabel = i;
        break;
      }
    }
  }
  Q_ASSERT(originLabel != -1);

  for (LeaderElectionNode* node : nextNbr->nodes) {
    if (node->prevNodeDir == originLabel) {
      if (nextNodeClone && recursion) {
        return node->nextNode();
      }
      else{
        return node;
      }
    }
  }
  Q_ASSERT(nextNbr->nodes.size() == 0);
  return nullptr;
}

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode*
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::prevNode(bool recursion) const {
  if (prevNodeDir < 0) {
    for (int i = 0; i < particle->nodes.size(); i++) {
      if (particle->nodes.at(i)->nodeDir == (nodeDir + 1) % 6) {
        return particle->nodes.at(i);
      }
    }
  }
  Q_ASSERT(0 <= prevNodeDir < 6);

  LeaderElectionStationaryDeterministicParticle* prevNbr =
      &particle->nbrAtLabel(prevNodeDir);
  
  int originLabel = -1;
  for (int i = 0; i < 6; i++) {
    if (prevNbr->hasNbrAtLabel(i)) {
      if (&prevNbr->nbrAtLabel(i) == particle) {
        originLabel = i;
        break;
      }
    }
  }
  Q_ASSERT(originLabel >= 0);

  for (LeaderElectionNode* node : prevNbr->nodes) {
    if (node->nextNodeDir == originLabel) {
      if (prevNodeClone && recursion) {
        return node->prevNode(false);
      }
      else {
        return node;
      }
    }
  }
  Q_ASSERT(prevNbr->nodes.size() == 0);
  return nullptr;
}

//----------------------------END AGENT CODE----------------------------

//----------------------------BEGIN SYSTEM CODE----------------------------

LeaderElectionStationaryDeterministicSystem::LeaderElectionStationaryDeterministicSystem(int numParticles) {
  Q_ASSERT(numParticles > 0);

  double holeProb = 0.0;

  // Insert the seed at (0,0).
  insert(new LeaderElectionStationaryDeterministicParticle(
      Node(0, 0), -1, randDir(), *this,
      LeaderElectionStationaryDeterministicParticle::State::IdentificationLabeling));
  std::set<Node> occupied;
  occupied.insert(Node(0, 0));

  int added = 1;
  while (added < numParticles) {
    for (Node n : occupied) {
      int dir = randDir();
      auto nbr = n.nodeInDir(dir);
      if (occupied.find(nbr) == occupied.end()) {
        int switches = 0;
        auto tmp = nbr.nodeInDir((dir + 5) % 6);
        bool lastOcc = occupied.find(tmp) != occupied.end();
        for (int count = 0; count < 6; ++count) {
          int i = (count + dir) % 6;
          auto nbrNbr = nbr.nodeInDir(i);
          if (occupied.find(nbrNbr) != occupied.end()) {
            if (!lastOcc) {
              ++switches;
            }
            lastOcc = true;
          } else {
            if (lastOcc) {
              ++switches;
            }
            lastOcc = false;
          }
        }
        if (switches <= 2) {
          occupied.insert(nbr);
          insert(new LeaderElectionStationaryDeterministicParticle(
              nbr, -1, randDir(), *this,
              LeaderElectionStationaryDeterministicParticle::State::IdentificationLabeling));
          ++added;
          if (added == numParticles) {
            break;
          }
        }
      }
    }
  }
}

bool LeaderElectionStationaryDeterministicSystem::hasTerminated() const {
#ifdef QT_DEBUG
  if (!isConnected(particles)) {
    return true;
  }
#endif

  for (auto p : particles) {
    auto hp = dynamic_cast<LeaderElectionStationaryDeterministicParticle *>(p);
    if (hp->state == LeaderElectionStationaryDeterministicParticle::State::Leader) {
      return true;
    }
  }

  return false;
}