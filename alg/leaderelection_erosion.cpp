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
                stateStable = false;
                return;
            }
        }
        else {
            // 3. Handedness agreement phase.
            // Candidate leaders (roots) agree on handedness first.
            // Process then proceeds through the forest from parents to children.
            qDebug() << "Particle: " << this->head.x << ", " << this->head.y;

            if (numCandidates == 0) {
                numCandidates = getNumCandidates();
            }
            qDebug() << "Adjacent candidates: " << numCandidates;

            if (numCandidates == 1) {
                // If there is 1 adjacent candidate (i.e. there are 2 candidates),
                // Let this particle be p, and call the other candidate q.
                // Get 2 vertices that are in common neighbours between p and q.
                // Call these u and v.
                int dir;
                for (auto c : candidates) {
                    dir = c;
                    break;
                }

                if (!isContracted()) {
                    dir = dirToTailLabel(dir);
                }

                qDebug() << "Getting neighbour at dir:" << dir;

                LeaderElectionErosionParticle& q = nbrAtLabel(dir);

                qDebug() << "Success.";

                int dir_u = (dir + 5) % 6;
                int dir_v = (dir + 1) % 6;
                if (numNbrsCandidate == -1) {
                    if (hasNbrAtLabel(dir_u) && hasNbrAtLabel(dir_v)) {
                        numNbrsCandidate = 2;
                    }
                    else if (hasNbrAtLabel(dir_u) || hasNbrAtLabel(dir_v)) {
                        numNbrsCandidate = 1;
                    }
                    else {
                        numNbrsCandidate = 0;
                    }
                    if (hasTailAtLabel(dir)) {
                        numNbrsCandidate = numNbrsCandidate - 1;
                    }
                }

                qDebug() << "Non-candidate neighbours: " << numNbrsCandidate;

                updateStability();
                qDebug() << "Stable: " << stable;
                if (!stable) {
                    stateStable = true;
                    return;
                }

                if (numNbrsCandidate == 2) {
                    // If both u and v are occupied
                    LeaderElectionErosionParticle& u = nbrAtLabel(dir_u);
                    LeaderElectionErosionParticle& v = nbrAtLabel(dir_v);
                    if (!chooseTokenSent) {
                        if ((dir - dir_u + 6) % 6 == 1) {
                            // Have u choose.
                            int globalizedDirU = localToGlobalDir(dir_u);
                            u.putToken(std::make_shared<YouChooseToken>(globalizedDirU));

                            int globalizedDirV = localToGlobalDir(dir_v);
                            v.putToken(std::make_shared<YouDoNotChooseToken>(globalizedDirV));

                            chooseTokenSent = true;
                        }
                        else {
                            // Have v choose.
                            int globalizedDirV = localToGlobalDir(dir_v);
                            v.putToken(std::make_shared<YouChooseToken>(globalizedDirV));

                            int globalizedDirU = localToGlobalDir(dir_u);
                            u.putToken(std::make_shared<YouDoNotChooseToken>(globalizedDirU));

                            chooseTokenSent = true;
                        }
                    }
                    else if (countTokens<SameHandednessToken>() == 2 || sameHandedness) {
                        if (!sameHandedness) {
                            takeToken<SameHandednessToken>();
                            takeToken<SameHandednessToken>();
                            sameHandedness = true;
                        }

                        // Agreed on handedness with adjacent candidate.
                        // Proceed to impose handedness on tree.
                        // TODO ...

                        qDebug() << "Agreed on handedness.";

                    }
                    stateStable = true;
                    return;
                }
                else if (numNbrsCandidate == 1) {
                    // If one of u, v is occupied, have the occupied particle choose.
                    if (hasNbrAtLabel(dir_u)) {
                        LeaderElectionErosionParticle& u = nbrAtLabel(dir_u);
                        if (!chooseTokenSent) {
                            int globalizedDir = localToGlobalDir(dir_u);
                            u.putToken(std::make_shared<YouChooseToken>(globalizedDir));
                            chooseTokenSent = true;
                        }
                        else {
                            if (hasToken<ChosenToken>()) {
                                if (hasToken<ParentToken>()) {
                                    int globalParentDir = takeToken<ParentToken>()->origin;
                                    int localParentDir = globalToLocalDir(globalParentDir);
                                    int localChildDir = (localParentDir + 3) % 6;
                                    children.insert(localChildDir);

                                    int globalChosenDir = takeToken<ChosenToken>()->origin;
                                    int localChosenDir = globalToLocalDir(globalChosenDir);
                                    int localNbrDir = (localChosenDir + 3) % 6;
                                    if (localNbrDir == dir_u) {
                                        state = State::Leader;
                                        stateStable = false;
                                        return;
                                    }
                                }
                            }
                            else if (hasToken<NotChosenToken>()) {
                                int globalChosenDir = takeToken<NotChosenToken>()->origin;
                                int localChosenDir = globalToLocalDir(globalChosenDir);
                                int localNbrDir = (localChosenDir + 3) % 6;
                                if (localNbrDir == dir_u) {
                                    state = State::Tree;
                                    parent = dir;
                                    int globalizedDir = localToGlobalDir(parent);
                                    q.putToken(std::make_shared<ParentToken>(globalizedDir));
                                    stateStable = false;
                                    return;
                                }
                            }
                        }
                    }
                    else {
                        LeaderElectionErosionParticle& v = nbrAtLabel(dir_v);
                        if (!chooseTokenSent) {
                            int globalizedDir = localToGlobalDir(dir_v);
                            v.putToken(std::make_shared<YouChooseToken>(globalizedDir));
                            chooseTokenSent = true;
                        }
                        else {
                            if (hasToken<ChosenToken>()) {
                                if (hasToken<ParentToken>()) {
                                    int globalParentDir = takeToken<ParentToken>()->origin;
                                    int localParentDir = globalToLocalDir(globalParentDir);
                                    int localChildDir = (localParentDir + 3) % 6;
                                    children.insert(localChildDir);

                                    int globalChosenDir = takeToken<ChosenToken>()->origin;
                                    int localChosenDir = globalToLocalDir(globalChosenDir);
                                    int localNbrDir = (localChosenDir + 3) % 6;
                                    if (localNbrDir == dir_v) {
                                        state = State::Leader;
                                        stateStable = false;
                                        return;
                                    }
                                }
                            }
                            else if (hasToken<NotChosenToken>()) {
                                int globalChosenDir = takeToken<NotChosenToken>()->origin;
                                int localChosenDir = globalToLocalDir(globalChosenDir);
                                int localNbrDir = (localChosenDir + 3) % 6;
                                if (localNbrDir == dir_v) {
                                    state = State::Tree;
                                    parent = dir;
                                    int globalizedDir = localToGlobalDir(parent);
                                    q.putToken(std::make_shared<ParentToken>(globalizedDir));
                                    stateStable = false;
                                    return;
                                }
                            }
                        }
                    }
                }
                else {
                    // If both u and v are unoccupied, use movement.
                    qDebug() << "Both unoccupied.";
                    if (isContracted()) {
                        qDebug() << "Contracted";
                    }
                    else {
                        qDebug() << "Expanded";
                    }
                    
                    if ((dir - dir_u + 6) % 6 == 1) {
                        // Attempt to move to u.
                        if (isContracted() && !hasMoved) {
                            if (canExpand(dir_u) && !hasToken<YouAreEliminatedToken>()) {
                                expand(dir_u);
                            }
                            else if (hasToken<YouAreEliminatedToken>()) {
                                takeToken<YouAreEliminatedToken>();
                                int globalizedDir = localToGlobalDir(dir);
                                q.putToken(std::make_shared<IAmEliminatedToken>(globalizedDir));
                                state = State::Tree;
                                parent = dir;
                                q.putToken(std::make_shared<ParentToken>(globalizedDir));
                                stateStable = false;
                                return;
                            }
                            else {
                                // Not received eliminated token yet, wait...
                                stateStable = true;
                                return;
                            }
                        }
                        else if (!isContracted() && !hasMoved) {
                            qDebug() << "has moved";
                            hasMoved = true;
                            int globalizedDir;
                            for (auto c : candidates) {
                                globalizedDir = c;
                                break;
                            }
                            globalizedDir = localToGlobalDir(globalizedDir);
                            if (hasTailAtLabel(dir) || !hasNbrAtLabel(dir)) {
                                // Same handedness
                                qDebug() << "same handedness";
                                sameHandedness = true;
                                q.putToken(std::make_shared<SameHandednessToken>(globalizedDir));
                                stateStable = true;
                                return;
                            }
                            else {
                                qDebug() << "eliminated";
                                q.putToken(std::make_shared<YouAreEliminatedToken>(globalizedDir));
                                stateStable = true;
                                return;
                            }
                        }
                        else if (hasMoved && !sameHandedness) {
                            if (hasToken<IAmEliminatedToken>() && hasToken<ParentToken>()) {
                                takeToken<IAmEliminatedToken>();
                                takeToken<ParentToken>();
                                children.insert(dir);
                                contractHead();
                                state = State::Leader;
                                stateStable = false;
                                return;
                            }
                        }
                        else if (hasMoved && sameHandedness && !isContracted()) {
                            if (hasToken<SameHandednessToken>()) {
                                takeToken<SameHandednessToken>();
                                contractHead();
                                stateStable = true;
                                return;
                            }
                        }
                        else if (hasMoved && sameHandedness && isContracted()) {
                            // Agreed on handedness with adjacent candidate.
                            // Proceed to impose handedness on tree.
                            // TODO ...

                            qDebug() << "Agreed on handedness.";
                        }
                    }
                    else {
                        // Attempt to move to v.
                        if (isContracted() && !hasMoved) {
                            if (canExpand(dir_v) && !hasToken<YouAreEliminatedToken>()) {
                                expand(dir_v);
                            }
                            else if (hasToken<YouAreEliminatedToken>()) {
                                takeToken<YouAreEliminatedToken>();
                                int globalizedDir = localToGlobalDir(dir);
                                q.putToken(std::make_shared<IAmEliminatedToken>(globalizedDir));
                                state = State::Tree;
                                parent = dir;
                                q.putToken(std::make_shared<ParentToken>(globalizedDir));
                                stateStable = false;
                                return;
                            }
                            else {
                                // Not received eliminated token yet, wait...
                                stateStable = true;
                                return;
                            }
                        }
                        else if (!isContracted() && !hasMoved) {
                            qDebug() << "has moved.";
                            hasMoved = true;
                            int globalizedDir;
                            for (auto c : candidates) {
                                globalizedDir = c;
                                break;
                            }
                            globalizedDir = localToGlobalDir(globalizedDir);
                            if (hasTailAtLabel(dir) || !hasNbrAtLabel(dir)) {
                                // Same handedness
                                sameHandedness = true;
                                q.putToken(std::make_shared<SameHandednessToken>(globalizedDir));
                                stateStable = true;
                                return;
                            }
                            else {
                                q.putToken(std::make_shared<YouAreEliminatedToken>(globalizedDir));
                                stateStable = true;
                                return;
                            }
                        }
                        else if (hasMoved && !sameHandedness) {
                            if (hasToken<IAmEliminatedToken>() && hasToken<ParentToken>()) {
                                takeToken<IAmEliminatedToken>();
                                takeToken<ParentToken>();
                                children.insert(dir);
                                contractHead();
                                state = State::Leader;
                                stateStable = false;
                                return;
                            }
                        }
                        else if (hasMoved && sameHandedness && !isContracted()) {
                            if (hasToken<SameHandednessToken>()) {
                                takeToken<SameHandednessToken>();
                                contractHead();
                                stateStable = true;
                                return;
                            }
                        }
                        else if (hasMoved && sameHandedness && isContracted()) {
                            // Agreed on handedness with adjacent candidate.
                            // Proceed to impose handedness on tree.
                            // TODO ...

                            qDebug() << "Agreed on handedness.";
                        }
                    }
                    stateStable = true;
                    return;
                }
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
        } // 3. Handedness agreement phase.
        else {
            // Either the 2 candidates have only this neighbour,
            // Or they have different handedness.
            // Choose one of them arbitrarily 
            // (just pick the one with the lowest label)
            if (countTokens<YouChooseToken>() == 2) {
                int globalDirP = takeToken<YouChooseToken>()->origin;
                int localDirP = globalToLocalDir(globalDirP);
                int dir_p = (localDirP + 3) % 6;
                LeaderElectionErosionParticle& p = nbrAtLabel(dir_p);

                int globalDirQ = takeToken<YouChooseToken>()->origin;
                int localDirQ = globalToLocalDir(globalDirQ);
                int dir_q = (localDirQ + 3) % 6;
                LeaderElectionErosionParticle& q = nbrAtLabel(dir_q);

                for (int dir = 0; dir < 6; dir++) {
                    if (dir == dir_p) {
                        int globalizedDirP = localToGlobalDir(dir_p);
                        p.putToken(std::make_shared<ChosenToken>(globalizedDirP));
                        int globalizedDirQ = localToGlobalDir(dir_q);
                        q.putToken(std::make_shared<NotChosenToken>(globalizedDirQ));
                        break;
                    }
                    else if (dir == dir_q) {
                        int globalizedDirQ = localToGlobalDir(dir_q);
                        q.putToken(std::make_shared<ChosenToken>(globalizedDirQ));
                        int globalizedDirP = localToGlobalDir(dir_p);
                        p.putToken(std::make_shared<NotChosenToken>(globalizedDirP));
                        break;
                    }
                }
            } // 2 candidates have the same handedness.
            else if (countTokens<YouChooseToken>() == 1 && countTokens<YouDoNotChooseToken>() == 1) {
                int globalDirP = takeToken<YouChooseToken>()->origin;
                int localDirP = globalToLocalDir(globalDirP);
                int dir_p = (localDirP + 3) % 6;
                LeaderElectionErosionParticle& p = nbrAtLabel(dir_p);

                int globalDirQ = takeToken<YouDoNotChooseToken>()->origin;
                int localDirQ = globalToLocalDir(globalDirQ);
                int dir_q = (localDirQ + 3) % 6;
                LeaderElectionErosionParticle& q = nbrAtLabel(dir_q);

                int globalizedDirP = localToGlobalDir(dir_p);
                p.putToken(std::make_shared<SameHandednessToken>(globalizedDirP));

                int globalizedDirQ = localToGlobalDir(dir_q);
                q.putToken(std::make_shared<SameHandednessToken>(globalizedDirQ));
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

int LeaderElectionErosionParticle::getNumCandidates() {
    int num = 0;
    for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle& nbr = nbrAtLabel(dir);
            if (nbr.state == State::Candidate || nbr.state == State::Root) {
                if (nbr.isContracted()) {
                    candidates.insert(dir);
                }
                else {
                    if (hasTailAtLabel(dir)) {
                        candidates.insert(dir);
                    }
                }
                if (hasHeadAtLabel(dir)) {
                    num = num + 1;
                }
            }
        }
    }
    return num;
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
