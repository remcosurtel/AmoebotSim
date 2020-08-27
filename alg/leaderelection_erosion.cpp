/* 
 * By Remco Surtel
 */

#include "alg/leaderelection_erosion.h"

#include <set>

#include <QtGlobal>

#include <QDebug>

using namespace std;

 //----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionErosionParticle::LeaderElectionErosionParticle(const Node head,
                                                             const int globalTailDir,
                                                             const int orientation,
                                                             AmoebotSystem& system,
                                                             State state,
                                                             int cornerType,
                                                             bool stable,
                                                             bool stateStable,
                                                             bool treeDone)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state), 
    cornerType(cornerType), stable(stable), stateStable(stateStable), treeDone(treeDone) {
    
}

void LeaderElectionErosionParticle::activate() {
    // 1. Lattice consumption phase.
    if (state == State::Eligible) {
        /* Determine the number of neighbours of the current particle.
         * If there are no neighbours, then that means the particle is the only
         * one in the system and should declare itself as the leader.
         * Otherwise, the particle will participate in leader election.
         */
        int numNbrs = getNumberOfNbrs();
        if (numNbrs == 0) {
            state = State::Leader;
            stateStable = false;
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

            // Update 'stable' flag according to neighbours.
            updateStability();

            // Update internal cornerType 
            cornerType = getCornerType();

            // If cornerType for some neighbour is not known, wait.
            for (int dir = 0; dir < 6; dir++) {
                if (hasNbrAtLabel(dir)) {
                    LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                    if (nbr.cornerType == -2) {
                        stateStable = true;
                        return;
                    }
                }
            }

            // If stable flag is not set, wait.
            if (!stable) {
                stateStable = true;
                return;
            }

            // If locked, wait.
            if (isLocked()) {
                stateStable = true;
                return;
            }

            if (cornerType < 0) {
                // Not a corner particle -> wait.
                stateStable = true;
                return;
            }
            else if (cornerType == 0) {
                // One 0-corner particle remaining.
                state = State::Candidate;
                stateStable = false;
                return;
            }
            else if (cornerType == 1) {
                for (int dir = 0; dir < 6; dir++) {
                    if (hasNbrAtLabel(dir)) {
                        LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                        if (nbr.state != State::Eroded) {
                            // If unique elible neighbour is 1-corner -> candidate
                            // Otherwise erode.
                            if (nbr.cornerType == 1) {
                                state = State::Candidate;
                                stateStable = false;
                                return;
                            }
                            else {
                                state = State::Eroded;
                                stateStable = false;
                                return;
                            }
                        }
                    }
                }
            }
            else if (cornerType == 2) {
                // If both eligible neighbours are 2-corner particles -> candidate
                // Otherwise erode.
                for (int dir = 0; dir < 6; dir++) {
                    if (hasNbrAtLabel(dir)) {
                        LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                        if (nbr.state != State::Eroded) {
                            if (nbr.cornerType != 2) {
                                state = State::Eroded;
                                stateStable = false;
                                return;
                            }
                        }
                    }
                }
                state = State::Candidate;
                stateStable = false;
                return;
            }
            else {
                // 3-corner particle -> erode.
                state = State::Eroded;
                stateStable = false;
                return;
            }
        }
    }
    else if (state == State::Candidate) {
        // Update 'stable' flag according to neighbours.
        updateStability();

        // Update internal cornerType 
        cornerType = getCornerType();

        // If stable flag is not set, wait.
        if (!stable) {
            stateStable = true;
            return;
        }

        if (cornerType == 0) {
            // If unique 0-corner candidate -> become leader.
            state = State::Leader;
            stateStable = false;
            return;
        } // Else: move to phase 2: spanning forest construction phase.
        else {
            // If other candidates have not set their candidate flag yet, wait.
            for (int dir = 0; dir < 6; dir++) {
                if (hasNbrAtLabel(dir)) {
                    LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                    if (nbr.state == State::Eligible) {
                        stateStable = true;
                        return;
                    }
                }
            }
            state = State::Root;
            parent = -1;
            stateStable = false;
            return;
        }
    }
    else if (state == State::Root) {
        // 2. Spanning forest construction phase.
        // Root receives parent tokens from neighbours.
        // Once all children set treeDone to true, the phase is finished.
        if (!treeDone) {
            while (hasToken<ParentToken>()) {
                int globalParentDir = takeToken<ParentToken>()->origin;
                int localParentDir = globalToLocalDir(globalParentDir);
                int localChildDir = (localParentDir + 3) % 6;
                children.insert(localChildDir);
            }
            if (treeIsDone()) {
                treeDone = true;
            }
        }
        stateStable = true;
        return;
    }
    else if (state == State::Eroded) {
        // 2. Spanning forest construction phase.
        // If a neighbour joins a tree, become its child in the tree.
        for (int dir = 0; dir < 6; dir++) {
            if (hasNbrAtLabel(dir)) {
                LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                if (nbr.state == State::Root) {
                    state = State::Tree;
                    parent = dir;
                    int globalizedDir = localToGlobalDir(parent);
                    nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
                    stateStable = false;
                    return;
                }
                else if (nbr.state == State::Tree) {
                    state = State::Tree;
                    parent = dir;
                    int globalizedDir = localToGlobalDir(parent);
                    nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
                    stateStable = false;
                    return;
                }
            }
        }
        stateStable = true;
        return;
    }
    else if (state == State::Tree) {
        // 2. Spanning forest construction phase.
        // Tree is done if all neighbours are in the tree
        // and all children are done
        if (!treeDone) {
            while (hasToken<ParentToken>()) {
                int globalParentDir = takeToken<ParentToken>()->origin;
                int localParentDir = globalToLocalDir(globalParentDir);
                int localChildDir = (localParentDir + 3) % 6;
                children.insert(localChildDir);
            }
            if (treeIsDone()) {
                treeDone = true;
            }
        }
        stateStable = true;
        return;
    }
    else {
        stateStable = true;
        return;
    }
}

bool LeaderElectionErosionParticle::treeIsDone() const {
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
            if (children.find(dir) != children.end()) {
                if (!nbr.treeDone) {
                    return false;
                }
            }
            else if (nbr.state != State::Tree && nbr.state != State::Root){
                return false;
            }
        }
    }
    return true;
}

bool LeaderElectionErosionParticle::isLocked() const {
    if (cornerType == 3) {
        // initialize array with integer for each neighbour.
        int nbrs[6] = {};
        for (int dir = 0; dir < 6; dir++) {
            if (hasNbrAtLabel(dir)) {
                LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
                if (nbr.state != State::Eroded) {
                    nbrs[dir] = nbr.cornerType;
                }
                else {
                    nbrs[dir] = -3;
                }
            }
            else {
                nbrs[dir] = -4;
            }
        }
        int eligibleNbrs = 0;
        for (int i = 0; i < 6; i++) {
            eligibleNbrs = eligibleNbrs + nbrs[i];
        }
        if (eligibleNbrs != 3) {
            return false;
        }
        else if (eligibleNbrs == 3) {
            for (int i = 0; i < 6; i++) {
                if (nbrs[i] >= -2) {
                    if (i > 0) {
                        if (nbrs[i + 1] >= -2 && nbrs[i + 2] >= -2) {
                            return nbrs[i + 1] == 3;
                        }
                        else {
                            return false;
                        }
                    }
                    else {
                        if (nbrs[i + 1] >= -2 && nbrs[i + 2] >= -2) {
                            return nbrs[i + 1] == 3;
                        }
                        else if (nbrs[i + 1] >= -2 && nbrs[5] >= -2) {
                            return nbrs[i] == 3;
                        }
                        else if (nbrs[5] >= -2 && nbrs[4] >= -2) {
                            return nbrs[5] == 3;
                        }
                        else {
                            return false;
                        }
                    }
                }
            }
        }
    }
    else {
        return false;
    }
}

void LeaderElectionErosionParticle::updateStability() {
    stable = true;
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
            if (!nbr.stateStable) {
                stable = false;
            }
        }
    }
}

int LeaderElectionErosionParticle::headMarkDir() const {
    if (state == State::Tree) {
        return parent;
    }
    else {
        return -1;
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
    else if (state == State::Root) {
        if (treeDone) {
            return 0x5a2d00;
        }
        return 0xc46200;
    }
    else if (state == State::Tree) {
        if (treeDone) {
            return 0x006100;
        }
        return 0x00b000;
    }
    else if (isLocked()) {
        return 0xfff000;
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
            LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
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
        LeaderElectionErosionParticle::State::Eligible, -2, false, false, false));
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
                    }
                    else {
                        if (lastOcc) {
                            ++switches;
                        }
                        lastOcc = false;
                    }
                }
                if (switches <= 2) {
                    occupied.insert(nbr);
                    insert(new LeaderElectionErosionParticle(nbr, -1, randDir(), *this,
                        LeaderElectionErosionParticle::State::Eligible, -2, false, false, false));
                    ++added;
                    if (added == numParticles) {
                        break;
                    }
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
        if (hp->state == LeaderElectionErosionParticle::State::Leader) {
            return true;
        }
    }

    return false;
}
