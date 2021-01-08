/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * S-contraction leader election.
 * Based on the paper "Distributed Leader Election and Computation of Local Identifiers
 *                     for Programmable Matter"
 * By Nicolas Gastineau, Wahabou Abdou, Nader Mbarek, and  Olivier Togni.
 * https://arxiv.org/abs/1807.10461 */

#include "alg/leaderelection_s-contraction.h"

#include <set>

#include <QtGlobal>

#include <QDebug>

using namespace std;

//----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionSContractionParticle::LeaderElectionSContractionParticle(
    const Node head, const int globalTailDir, const int orientation,
    AmoebotSystem &system, State state)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state) {}

void LeaderElectionSContractionParticle::activate() {
  if (state == State::Candidate) {
    if (isSContractible()) {
      if (!hasCandidateNbr()) {
        state = State::Leader;
      }
      else {
        state = State::NotElected;
      }
    }
  }
}

int LeaderElectionSContractionParticle::headMarkDir() const {
  return -1;
}

int LeaderElectionSContractionParticle::headMarkColor() const {
  if (state == State::NotElected) {
    return 0x7e7e7e; // gray
  }
  else if (state == State::Leader) {
    return 0x00ff00; // green
  }
  return -1;
}

LeaderElectionSContractionParticle &
LeaderElectionSContractionParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<LeaderElectionSContractionParticle>(label);
}

QString LeaderElectionSContractionParticle::inspectionText() const {
  QString text;
  QString indent = "    ";

  text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
          ")\n";
  text += "orientation: " + QString::number(orientation) + "\n";
  text += "globalTailDir: " + QString::number(globalTailDir) + "\n";
  text += "state: ";
  text += [this]() {
    switch (state) {
    case State::Leader:
      return "leader";
    case State::NotElected:
      return "not elected";
    case State::Candidate:
      return "candidate";
    default:
      return "no state";
    }
  }();
  text += "\n";

  return text;
}

bool LeaderElectionSContractionParticle::isSContractible() {
  if (candidatesConnected() && nonCandidateAdjacent()) {
    return true;
  }
  else {
    return false;
  }
}

bool LeaderElectionSContractionParticle::candidatesConnected() {
  for (int dir = 0; dir < 6; dir++) {
    bool nonCandidate = false;
    if (hasNbrAtLabel(dir)) {
      LeaderElectionSContractionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state != State::Candidate) {
        nonCandidate = true;
      }
    }
    else {
      nonCandidate = true;
    }

    if (nonCandidate) {
      for (int dir_2 = dir+1; dir_2 < 6; dir_2++) {
        nonCandidate = false;
        if (hasNbrAtLabel(dir_2)) {
          LeaderElectionSContractionParticle &nbr_2 = nbrAtLabel(dir_2);
          if (nbr_2.state != State::Candidate) {
            nonCandidate = true;
          }
        }
        else {
          nonCandidate = true;
        }

        if (nonCandidate) {
          bool candidateLeft = false;
          int i = (dir + 1) % 6;
          while (i != dir_2) {
            if (hasNbrAtLabel(i)) {
              LeaderElectionSContractionParticle &p = nbrAtLabel(i);
              if (p.state == State::Candidate) {
                candidateLeft = true;
                break;
              }
            }
            i = (i + 1) % 6;
          }
          bool candidateRight = false;
          i = (dir + 5) % 6;
          while (i != dir_2) {
            if (hasNbrAtLabel(i)) {
              LeaderElectionSContractionParticle &p = nbrAtLabel(i);
              if (p.state == State::Candidate) {
                candidateRight = true;
                break;
              }
            }
            i = (i + 5) % 6;
          }

          if (candidateLeft && candidateRight) {
            return false;
          }
        }
      }
    }
  }
  return true;
}

bool LeaderElectionSContractionParticle::nonCandidateAdjacent() {
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionSContractionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state != State::Candidate) {
        return true;
      }
    }
    else {
      return true;
    }
  }
  return false;
}

bool LeaderElectionSContractionParticle::hasCandidateNbr() {
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionSContractionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state == State::Candidate) {
        return true;
      }
    }
  }
  return false;
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN SYSTEM CODE----------------------------

#include <string>
#include <fstream>
#include <sstream>
#include <QTextStream>

using namespace std;

LeaderElectionSContractionSystem::LeaderElectionSContractionSystem(int numParticles, QString fileName) {
  Q_ASSERT(numParticles > 0 || fileName.size() > 0);

  string filePath = "../AmoebotSim/data/input/" + fileName.toStdString() + ".txt";
  if (fileName != "") {
    QTextStream out(stdout);
    out << "File name: " << fileName << endl;
    ifstream file(filePath);
    if (!file) {
      out << "Cannot open file." << endl;
      return;
    }
    out << "File opened." << endl;
    
    string str;
    while (getline(file, str)) {
      std::vector<int> vect;
      std::stringstream ss(str);

      while (ss.good()) {
        string substr;
        getline(ss, substr, ',');
        vect.push_back(std::stoi(substr));
      }

      int x = vect[0];
      int y = vect[1];

      insert(new LeaderElectionSContractionParticle(
      Node(x, y), -1, randDir(), *this,
      LeaderElectionSContractionParticle::State::Candidate));
    }

    file.close();

    outputPath = "../AmoebotSim/data/output/" + fileName.toStdString() + ".txt";

    out << "Particle system initialized from file." << endl;
    
    return;
  }

  randomPermutationScheduler = true;
  // randomReshuffleProb = 0.1;

  // Insert the seed at (0,0).
  insert(new LeaderElectionSContractionParticle(
      Node(0, 0), -1, randDir(), *this,
      LeaderElectionSContractionParticle::State::Candidate));
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
          insert(new LeaderElectionSContractionParticle(
              nbr, -1, randDir(), *this,
              LeaderElectionSContractionParticle::State::Candidate));
          ++added;
          if (added == numParticles) {
            break;
          }
        }
      }
    }
  }
}

bool LeaderElectionSContractionSystem::hasTerminated() const {
#ifdef QT_DEBUG
  if (!isConnected(particles)) {
    return true;
  }
#endif

  for (auto p : particles) {
    auto hp = dynamic_cast<LeaderElectionSContractionParticle *>(p);
    if (hp->state == LeaderElectionSContractionParticle::State::Leader) {
      if (outputPath != "") {
        ofstream file;
        file.open(outputPath);
        file << std::to_string(hp->head.x) << "," << std::to_string(hp->head.y);

        file << "\n" << std::to_string(getCount("# Rounds")._value);
        file << "\n" << std::to_string(getCount("# Activations")._value);
        file << "\n" << std::to_string(getCount("# Moves")._value);

        file.close();

        QTextStream out(stdout);
        out << "Output written to: " << QString::fromStdString(outputPath) << endl;
      }
      return true;
    }
  }

  return false;
}