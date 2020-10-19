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
      qDebug() << "@@@@@@@@@@@@@@@@@@@@ Initializing nodes... @@@@@@@@@@@@@@@@@@@@";
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
      else if (node->predecessor == nullptr && node->successor == nullptr) {
        node->paintNode(0x00ff00); // Green
      }
      else if (node->predecessor == nullptr) {
        node->paintNode(0xff9b00);
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
      if (hasNodeToken<MergeNackToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<MergeNackToken>(nextNode()->prevNodeDir);
        mergePending = false;
      }
      if (hasNodeToken<MergeAckToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<MergeAckToken>(nextNode()->prevNodeDir);
        mergeAck = true;
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
            return;
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
          if (!countSent && count > 0) {
            qDebug() << "Sending count towards tail...";
            passNodeToken<CountToken>(nextNodeDir, std::make_shared<CountToken>(-1, count));
            countSent = true;
            qDebug() << "Count sent";
          }
          else {
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
              }
              else if (count > 0 && count == value && count + value <= 6) {
                qDebug() << "Counts equal, lexicographic comparison...";
                // Perform lexicographic comparison
                // TODO
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