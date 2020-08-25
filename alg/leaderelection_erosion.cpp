/* 
 * By Remco Surtel
 */

#include "alg/leaderelection_erosion.h"

#include <set>

#include <QtGlobal>

 //----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionErosionParticle::LeaderElectionErosionParticle(const Node head,
                                                             const int globalTailDir,
                                                             const int orientation,
                                                             AmoebotSystem& system,
                                                             State state)
    : AmoebotParticle(head, globalTailDir, orientation, system),
      state(State::Eligible) {
    
}

void LeaderElectionErosionParticle::activate() {
    if (state == State::Eligible) {
        /* Determine the number of neighbors of the current particle.
         * If there are no neighbors, then that means the particle is the only
         * one in the system and should declare itself as the leader.
         * Otherwise, the particle will participate in leader election.
         */
        int numNbrs = getNumberOfNbrs();
        if (numNbrs == 0) {
            state = State::Leader;
            return;
        }
        else {
            /*
             * Determine if this particle is a corner particle.
             * x-corner particles have 0 <= x <= 3 adjacent eligible neighbours.
             * If this particle is a corner particle, then erode.
             * Unless a termination condition holds.
             * Termination conditions:
             * - One 0-corner particle remains
             * - Two 1-corner particles remain
             * - Three 2-corner particles remain
             */
            int cornerType = getCornerType();
            if (cornerType < 0) {
                // Not a corner particle.
                return;
            }
            else if (cornerType == 0) {
                // One 0-corner particle remaining: leader.
                state = State::Leader;
                return;
            }
            else if (cornerType < 3) {
                // Check termination condition.
                bool terminate = true;
                for (int dir = 0; dir < 6; dir++) {
                    if (hasNbrAtLabel(dir)) {
                        LeaderElectionErosionParticle nbr = nbrAtLabel(dir);
                        if (nbr.state == State::Eligible) {
                            if (nbr.getCornerType() != cornerType) {
                                terminate = false;
                            }
                        }
                    }
                }
                // If termination condition holds, advance to next phase.
                if (terminate) {
                    state = State::Candidate;
                    return;
                }
                // If termination condition does not hold, erode.
                state = State::Eroded;
                return;
            }
            else {
                // 3-corner particle -> erode.
                state = State::Eroded;
                return;
            }
        }
    }
}

int LeaderElectionErosionParticle::headMarkColor() const {
    if (state == State::Leader) {
        return 0x00ff00;
    }
    else if (state == State::Candidate) {
        return 0xffaa00;
    }
    else if (state == State::Eroded) {
        return 0x999999;
    }
    else if (state == State::Finished) {
        return 0xffe000;
    }

    return -1;
}

LeaderElectionErosionParticle& LeaderElectionErosionParticle::nbrAtLabel(int label) const {
    return AmoebotParticle::nbrAtLabel<LeaderElectionErosionParticle>(label);
}

int LeaderElectionErosionParticle::getCornerType() const {
    int nbrs[6] = {};
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle nbr = nbrAtLabel(dir);
            if (nbr.state != State::Eroded) {
                nbrs[dir] = 1;
            }
            else {
                nbrs[dir] = 0;
            }
        }
        else {
            nbrs[dir] = 0;
        }
    }
        
    int eligibleNbrs = 0;
    for (int i = 0; i < 6; i++) {
        eligibleNbrs = eligibleNbrs + nbrs[i];
    }
    if (eligibleNbrs > 3) {
        return -1;
    }
    else if (eligibleNbrs == 0) {
        return 0;
    }
    else if (eligibleNbrs == 1) {
        return 1;
    }
    else if (eligibleNbrs == 2) {
        for (int i = 0; i < 6; i++) {
            if (nbrs[i] == 1) {
                if (i > 0) {
                    if (nbrs[i + 1] == 1) {
                        return 2;
                    }
                    else {
                        return -1;
                    }
                }
                else {
                    if (nbrs[i + 1] == 1 || nbrs[5] == 1) {
                        return 2;
                    }
                    else {
                        return -1;
                    }
                }
            }
        }
    }
    else if (eligibleNbrs == 3) {
        for (int i = 0; i < 6; i++) {
            if (nbrs[i] == 1) {
                if (i > 0) {
                    if (nbrs[i + 1] == 1 && nbrs[i + 2] == 1) {
                        return 3;
                    }
                    else {
                        return -1;
                    }
                }
                else {
                    if ((nbrs[i + 1] == 1 && nbrs[i + 2] == 1) || 
                        (nbrs[i + 1] == 1 && nbrs[5] == 1) || 
                        (nbrs[5] == 1 && nbrs[4] == 1)) {
                        return 3;
                    }
                    else {
                        return -1;
                    }
                }
            }
        }
    }
}

int LeaderElectionErosionParticle::getNumberOfNbrs() const {
    int count = 0;
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            count++;
        }
    }
    return count;
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN SYSTEM CODE----------------------------

LeaderElectionErosionSystem::LeaderElectionErosionSystem(int numParticles) {
    Q_ASSERT(numParticles > 0);
    
    double holeProb = 0.0;

    // Insert the seed at (0,0).
    insert(new LeaderElectionErosionParticle(Node(0, 0), -1, randDir(), *this,
        LeaderElectionErosionParticle::State::Eligible));
    std::set<Node> occupied;
    occupied.insert(Node(0, 0));

    std::set<Node> candidates;
    for (int i = 0; i < 6; ++i) {
        candidates.insert(Node(0, 0).nodeInDir(i));
    }

    // Add inactive particles.
    int numNonStaticParticles = 0;
    while (numNonStaticParticles < numParticles && !candidates.empty()) {
        // Pick random candidate.
        int randIndex = randInt(0, candidates.size());
        Node randomCandidate;
        for (auto it = candidates.begin(); it != candidates.end(); ++it) {
            if (randIndex == 0) {
                randomCandidate = *it;
                candidates.erase(it);
                break;
            }
            else {
                randIndex--;
            }
        }

        occupied.insert(randomCandidate);

        // Add this candidate as a particle if not a hole.
        if (randBool(1.0 - holeProb)) {
            insert(new LeaderElectionErosionParticle(randomCandidate, -1, randDir(), *this,
                LeaderElectionErosionParticle::State::Eligible));
            ++numNonStaticParticles;

            // Add new candidates.
            for (int i = 0; i < 6; ++i) {
                auto neighbor = randomCandidate.nodeInDir(i);
                if (occupied.find(neighbor) == occupied.end()) {
                    candidates.insert(neighbor);
                }
            }
        }
    }
}

bool LeaderElectionErosionSystem::hasTerminated() const {
#ifdef QT_DEBUG
    if (!isConnected(particles)) {
        return true;
    }
#endif

    for (auto p : particles) {
        auto hp = dynamic_cast<LeaderElectionErosionParticle*>(p);
        if (hp->state != LeaderElectionErosionParticle::State::Leader &&
            hp->state != LeaderElectionErosionParticle::State::Finished) {
            return false;
        }
    }

    return true;
}
