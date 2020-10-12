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
}

void LeaderElectionStationaryDeterministicParticle::activate() {
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
            }
            else {
              node->nextNodeDir = dir;
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
      state = State::StretchExpansion;
      return;
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
}

int LeaderElectionStationaryDeterministicParticle::headMarkColor() const {
  return -1;
}

LeaderElectionStationaryDeterministicParticle &
LeaderElectionStationaryDeterministicParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<LeaderElectionStationaryDeterministicParticle>(label);
}

QString LeaderElectionStationaryDeterministicParticle::inspectionText() const {
  QString text;
  QString indent = "    ";

  text = "";

  return text;
}

std::array<int, 18> LeaderElectionStationaryDeterministicParticle::borderColors() const {
  return borderColorLabels;
}

std::array<int, 6> LeaderElectionStationaryDeterministicParticle::borderPointColors() const {
  return borderPointColorLabels;
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
          LeaderElectionNode* next = nextNode();
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
            successor = nextNode();
            count += successor->count;
          }
          else {
            qDebug() << "Merging left...";
            predecessor = prevNode();
          }
          mergePending = false;
          mergeAck = false;
          qDebug() << "Merged.";
        }
      }
      else {
        if (mergePending && mergeAck && mergeDir == -1) {
          qDebug() << "Merging left...";
          predecessor = prevNode();
          mergePending = false;
          mergeAck = false;
          qDebug() << "Merged.";
        }
      }
    }
  }
}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::paintNode(
    const int color) {
  // paint a node
  particle->borderColorLabels.at(3 * particle->localToGlobalDir(nodeDir) + 2) = color;
}

template <class TokenType>
bool LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
hasNodeToken(int dir) const{
    auto prop = [dir](const std::shared_ptr<TokenType> token) {
      return token->origin == dir;
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
takeNodeToken(int dir) {
  auto prop = [dir](const std::shared_ptr<TokenType> token) {
    return token->origin == dir;
  };
  return particle->takeToken<TokenType>(prop);
}

template <class TokenType>
void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::
passNodeToken(int dir, std::shared_ptr<TokenType> token) {
  LeaderElectionStationaryDeterministicParticle* nbr;
  if (dir < 0) {
    if (dir == nextNodeDir) {
      nbr = nextNode()->particle;
    }
    else if (dir == prevNodeDir) {
      nbr = prevNode()->particle;
    }
  }
  else {
    nbr = &particle->nbrAtLabel(dir);
  }
  int origin = -1;
  if (dir >= 0) {
    for (int i = 0; i < 6; i++) {
      if (nbr->hasNbrAtLabel(i)) {
        if (&nbr->nbrAtLabel(i) == particle) {
          origin = i;
          break;
        }
      }
    }
    Q_ASSERT(origin != -1);
  }
  else {
    origin = dir;
  }
  token->origin = origin;
  nbr->putToken(token);
}

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode*
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::nextNode() const {
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
      return node;
    }
  }
  Q_ASSERT(nextNbr->nodes.size() == 0);
  return nullptr;
}

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode*
LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::prevNode() const {
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
  Q_ASSERT(originLabel != -1);

  for (LeaderElectionNode* node : prevNbr->nodes) {
    if (node->nextNodeDir == originLabel) {
      return node;
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