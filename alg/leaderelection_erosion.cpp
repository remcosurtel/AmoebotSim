/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Erosion-based leader election.
 * Based on the paper "Shape Formation by Programmable Particles"
 * By Giuseppe A. Di Luna, Paola Flocchini, Nicola Santoro, Giovanni Viglietta,
 * Yukiko Yamauchi https://arxiv.org/abs/1705.03538 */

#include "alg/leaderelection_erosion.h"

#include <set>

#include <QtGlobal>

#include <QDebug>

using namespace std;

//----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionErosionParticle::LeaderElectionErosionParticle(
    const Node head, const int globalTailDir, const int orientation,
    AmoebotSystem &system, State state)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state) {}

void LeaderElectionErosionParticle::activate() {
  // 1. Lattice consumption phase.
  if (state == State::Eligible) {
    /* Determine the number of neighbors of the current particle.
     * If there are no neighbors, then that means the particle is the only
     * one in the system and should declare itself as the leader.
     * Otherwise, the particle will participate in leader election. */
    int numNbrs = getNumberOfNbrs();
    if (numNbrs == 0) {
      state = State::Leader;
      stateStable = false;
      return;
    } else {
      /*
       * Determine if this particle is a corner particle.
       * x-corner particles have 0 <= x <= 3 adjacent eligible neighbors.
       * If this particle is a corner particle, then erode.
       * Unless a termination condition holds.
       * Termination conditions:
       * - One 0-corner particle remains
       * - Two 1-corner particles remain
       * - Three 2-corner particles remain */

      // Update 'stable' flag according to neighbors.
      updateStability();

      // Update internal cornerType
      cornerType = getCornerType();

      // If cornerType for some neighbor is not known, wait.
      for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
          LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
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
      } else if (cornerType == 0) {
        // One 0-corner particle remaining.
        state = State::Candidate;
        stateStable = false;
        return;
      } else if (cornerType == 1) {
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
            if (nbr.state != State::Eroded) {
              // If unique elible neighbor is 1-corner -> candidate
              // Otherwise erode.
              if (nbr.cornerType == 1) {
                state = State::Candidate;
                stateStable = false;
                return;
              } else {
                state = State::Eroded;
                stateStable = false;
                return;
              }
            }
          }
        }
      } else if (cornerType == 2) {
        // If both eligible neighbors are 2-corner particles -> candidate
        // Otherwise erode.
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
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
      } else {
        // 3-corner particle -> erode.
        state = State::Eroded;
        stateStable = false;
        return;
      }
    }
  } else if (state == State::Candidate) {
    // Update 'stable' flag according to neighbors.
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
          LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
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
  } else if (state == State::Root) {
    // 2. Spanning forest construction phase.
    // Root receives parent tokens from neighbors.
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
    } else {
      // 3. Handedness agreement phase.
      // Candidate leaders (roots) agree on handedness first.
      // Process then proceeds through the forest from parents to children.

      if (numCandidates == 0) {
        numCandidates = getNumCandidates();
      }

      if (sameHandedness) {
        updateStability();

        if (!stable) {
          stateStable = true;
          return;
        }

        state = State::RootElection;
        stateStable = false;
        return;
      }

      if (numCandidates == 1) {
        // If there is 1 adjacent candidate (i.e. there are 2 candidates),
        // Let this particle be p, and call the other candidate q.
        // Get 2 vertices that are in common neighbors between p and q.
        // Call these u and v.
        int dir;
        for (auto c : candidates) {
          dir = c;
          break;
        }

        if (!isContracted()) {
          dir = dirToTailLabel(dir);
        }

        LeaderElectionErosionParticle &q = nbrAtLabel(dir);

        int dir_u = (dir + 5) % 6;
        int dir_v = (dir + 1) % 6;
        if (numNbrsCandidate == -1) {
          if (hasNbrAtLabel(dir_u) && hasNbrAtLabel(dir_v)) {
            numNbrsCandidate = 2;
          } else if (hasNbrAtLabel(dir_u) || hasNbrAtLabel(dir_v)) {
            numNbrsCandidate = 1;
          } else {
            numNbrsCandidate = 0;
          }
          if (hasTailAtLabel(dir)) {
            numNbrsCandidate = numNbrsCandidate - 1;
          }
        }

        updateStability();

        if (!stable) {
          stateStable = true;
          return;
        }

        if (numNbrsCandidate == 2) {
          // If both u and v are occupied
          LeaderElectionErosionParticle &u = nbrAtLabel(dir_u);
          LeaderElectionErosionParticle &v = nbrAtLabel(dir_v);
          if (!chooseTokenSent) {
            if ((dir - dir_u + 6) % 6 == 1) {
              // Have u choose.
              int globalizedDirU = localToGlobalDir(dir_u);
              u.putToken(std::make_shared<YouChooseToken>(globalizedDirU));

              int globalizedDirV = localToGlobalDir(dir_v);
              v.putToken(std::make_shared<YouDoNotChooseToken>(globalizedDirV));

              chooseTokenSent = true;
            } else {
              // Have v choose.
              int globalizedDirV = localToGlobalDir(dir_v);
              v.putToken(std::make_shared<YouChooseToken>(globalizedDirV));

              int globalizedDirU = localToGlobalDir(dir_u);
              u.putToken(std::make_shared<YouDoNotChooseToken>(globalizedDirU));

              chooseTokenSent = true;
            }
          } else if (countTokens<SameHandednessToken>() == 2 ||
                     sameHandedness) {
            if (!sameHandedness) {
              takeToken<SameHandednessToken>();
              takeToken<SameHandednessToken>();
              sameHandedness = true;
            }

            // Agreed on handedness with adjacent candidate.
            // Proceed to impose handedness on tree.

            qDebug() << "Agreed on handedness.";

            state = State::RootElection;
            stateStable = false;
            return;
          }
          stateStable = true;
          return;
        } else if (numNbrsCandidate == 1) {
          // If one of u, v is occupied, have the occupied particle choose.
          if (hasNbrAtLabel(dir_u)) {
            LeaderElectionErosionParticle &u = nbrAtLabel(dir_u);
            if (!chooseTokenSent) {
              int globalizedDir = localToGlobalDir(dir_u);
              u.putToken(std::make_shared<YouChooseToken>(globalizedDir));
              chooseTokenSent = true;
            } else {
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
              } else if (hasToken<NotChosenToken>()) {
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
          } else {
            LeaderElectionErosionParticle &v = nbrAtLabel(dir_v);
            if (!chooseTokenSent) {
              int globalizedDir = localToGlobalDir(dir_v);
              v.putToken(std::make_shared<YouChooseToken>(globalizedDir));
              chooseTokenSent = true;
            } else {
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
              } else if (hasToken<NotChosenToken>()) {
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
        } else {
          // If both u and v are unoccupied, use movement.
          if ((dir - dir_u + 6) % 6 == 1) {
            // Attempt to move to u.
            if (isContracted() && !hasMoved) {
              if (canExpand(dir_u) && !hasToken<YouAreEliminatedToken>()) {
                expand(dir_u);
              } else if (hasToken<YouAreEliminatedToken>()) {
                takeToken<YouAreEliminatedToken>();
                int globalizedDir = localToGlobalDir(dir);
                q.putToken(std::make_shared<IAmEliminatedToken>(globalizedDir));
                state = State::Tree;
                parent = dir;
                q.putToken(std::make_shared<ParentToken>(globalizedDir));
                stateStable = false;
                return;
              } else {
                // Not received eliminated token yet, wait...
                stateStable = true;
                return;
              }
            } else if (!isContracted() && !hasMoved) {
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
                q.putToken(
                    std::make_shared<SameHandednessToken>(globalizedDir));
                stateStable = true;
                return;
              } else {
                q.putToken(
                    std::make_shared<YouAreEliminatedToken>(globalizedDir));
                stateStable = true;
                return;
              }
            } else if (hasMoved && !sameHandedness) {
              if (hasToken<IAmEliminatedToken>() && hasToken<ParentToken>()) {
                takeToken<IAmEliminatedToken>();
                takeToken<ParentToken>();
                children.insert(dir);
                contractHead();
                state = State::Leader;
                stateStable = false;
                return;
              }
            } else if (hasMoved && sameHandedness && !isContracted()) {
              if (hasToken<SameHandednessToken>()) {
                takeToken<SameHandednessToken>();
                contractHead();
                stateStable = true;
                return;
              }
            } else if (hasMoved && sameHandedness && isContracted()) {
              // Agreed on handedness with adjacent candidate.
              // Proceed to impose handedness on tree.

              qDebug() << "Agreed on handedness.";

              state = State::RootElection;
              stateStable = false;
              return;
            }
          } else {
            // Attempt to move to v.
            if (isContracted() && !hasMoved) {
              if (canExpand(dir_v) && !hasToken<YouAreEliminatedToken>()) {
                expand(dir_v);
              } else if (hasToken<YouAreEliminatedToken>()) {
                takeToken<YouAreEliminatedToken>();
                int globalizedDir = localToGlobalDir(dir);
                q.putToken(std::make_shared<IAmEliminatedToken>(globalizedDir));
                state = State::Tree;
                parent = dir;
                q.putToken(std::make_shared<ParentToken>(globalizedDir));
                stateStable = false;
                return;
              } else {
                // Not received eliminated token yet, wait...
                stateStable = true;
                return;
              }
            } else if (!isContracted() && !hasMoved) {
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
                q.putToken(
                    std::make_shared<SameHandednessToken>(globalizedDir));
                stateStable = true;
                return;
              } else {
                q.putToken(
                    std::make_shared<YouAreEliminatedToken>(globalizedDir));
                stateStable = true;
                return;
              }
            } else if (hasMoved && !sameHandedness) {
              if (hasToken<IAmEliminatedToken>() && hasToken<ParentToken>()) {
                takeToken<IAmEliminatedToken>();
                takeToken<ParentToken>();
                children.insert(dir);
                contractHead();
                state = State::Leader;
                stateStable = false;
                return;
              }
            } else if (hasMoved && sameHandedness && !isContracted()) {
              if (hasToken<SameHandednessToken>()) {
                takeToken<SameHandednessToken>();
                contractHead();
                stateStable = true;
                return;
              }
            } else if (hasMoved && sameHandedness && isContracted()) {
              // Agreed on handedness with adjacent candidate.
              // Proceed to impose handedness on tree.

              qDebug() << "Agreed on handedness.";

              state = State::RootElection;
              stateStable = false;
              return;
            }
          }
          stateStable = true;
          return;
        }
      } else if (numCandidates == 2) {
        // If there are 2 adjacent candidates (i.e. there are 3 candidates),
        // Call the other 2 candidates q and r.
        // Either 1 candidate has unique handedness and becomes the leader,
        // Or all 3 candidates have the same handedness and will impose
        // this handedness on their trees.

        int dir_q = -1;
        int dir_r;
        for (auto c : candidates) {
          if (dir_q == -1) {
            dir_q = c;
          } else {
            dir_r = c;
            break;
          }
        }

        LeaderElectionErosionParticle &q = nbrAtLabel(dir_q);
        LeaderElectionErosionParticle &r = nbrAtLabel(dir_r);

        int globalizedDirQ = localToGlobalDir(dir_q);
        int globalizedDirR = localToGlobalDir(dir_r);

        // If not chosen yet, choose one of the other candidates based on
        // handedness.
        if (!chooseTokenSent) {
          // choose q
          if (dir_r == (dir_q + 1) % 6) {
            q.putToken(std::make_shared<ChosenToken>(globalizedDirQ));
            r.putToken(std::make_shared<NotChosenToken>(globalizedDirR));
            chooseTokenSent = true;
          } // choose r
          else {
            q.putToken(std::make_shared<NotChosenToken>(globalizedDirQ));
            r.putToken(std::make_shared<ChosenToken>(globalizedDirR));
            chooseTokenSent = true;
          }
        }

        // If chosen twice, eliminate other candidates.
        if (countTokens<ChosenToken>() == 2) {
          takeToken<ChosenToken>();
          takeToken<ChosenToken>();
          q.putToken(std::make_shared<YouAreEliminatedToken>(globalizedDirQ));
          r.putToken(std::make_shared<YouAreEliminatedToken>(globalizedDirR));

          sameHandedness = true;

          state = State::Leader;
          stateStable = false;
          return;
        }
        // If chosen once and not chosen once, handedness is the same.
        else if (countTokens<ChosenToken>() == 1 &&
                 countTokens<NotChosenToken>() == 1) {
          takeToken<ChosenToken>();
          takeToken<NotChosenToken>();
          q.putToken(std::make_shared<IAmEliminatedToken>(globalizedDirQ));
          r.putToken(std::make_shared<IAmEliminatedToken>(globalizedDirR));
          notChosen = true;
        }

        // If eliminated, revoke candidacy and become child node of leader.
        if (hasToken<YouAreEliminatedToken>()) {
          int globalLeaderDir = takeToken<YouAreEliminatedToken>()->origin;
          int localLeaderDir = globalToLocalDir(globalLeaderDir);
          int localNbrDir = (localLeaderDir + 3) % 6;

          state = State::Tree;
          parent = localNbrDir;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &l = nbrAtLabel(parent);
          l.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        }
        // If no one was chosen, handedness is the same
        else if (countTokens<IAmEliminatedToken>() == 2 && notChosen) {
          takeToken<IAmEliminatedToken>();
          takeToken<IAmEliminatedToken>();
          sameHandedness = true;
        }

        if (sameHandedness) {
          // Agreed on handedness with adjacent candidates.
          // Proceed to impose handedness on tree.

          qDebug() << "Agreed on handedness.";

          state = State::RootElection;
          stateStable = false;
          return;
        }

        stateStable = true;
        return;
      }
    }
    stateStable = true;
    return;
  } else if (state == State::Eroded) {
    // 2. Spanning forest construction phase.
    // If a neighbor joins a tree, become its child in the tree.
    for (int dir = 0; dir < 6; dir++) {
      if (hasNbrAtLabel(dir)) {
        LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
        if (nbr.state == State::Root) {
          state = State::Tree;
          parent = dir;
          int globalizedDir = localToGlobalDir(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        } else if (nbr.state == State::Tree) {
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
  } else if (state == State::Tree) {
    // 2. Spanning forest construction phase.
    // Tree is done if all neighbors are in the tree
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
    } else {
      // 3. Handedness agreement phase.
      // Either the 2 candidates have only this neighbor,
      // Or they have different handedness.
      // Choose one of them arbitrarily
      // (just pick the one with the lowest label)
      if (countTokens<YouChooseToken>() == 2) {
        int globalDirP = takeToken<YouChooseToken>()->origin;
        int localDirP = globalToLocalDir(globalDirP);
        int dir_p = (localDirP + 3) % 6;
        LeaderElectionErosionParticle &p = nbrAtLabel(dir_p);

        int globalDirQ = takeToken<YouChooseToken>()->origin;
        int localDirQ = globalToLocalDir(globalDirQ);
        int dir_q = (localDirQ + 3) % 6;
        LeaderElectionErosionParticle &q = nbrAtLabel(dir_q);

        for (int dir = 0; dir < 6; dir++) {
          if (dir == dir_p) {
            int globalizedDirP = localToGlobalDir(dir_p);
            p.putToken(std::make_shared<ChosenToken>(globalizedDirP));
            int globalizedDirQ = localToGlobalDir(dir_q);
            q.putToken(std::make_shared<NotChosenToken>(globalizedDirQ));
            break;
          } else if (dir == dir_q) {
            int globalizedDirQ = localToGlobalDir(dir_q);
            q.putToken(std::make_shared<ChosenToken>(globalizedDirQ));
            int globalizedDirP = localToGlobalDir(dir_p);
            p.putToken(std::make_shared<NotChosenToken>(globalizedDirP));
            break;
          }
        }
      } // 2 candidates have the same handedness.
      else if (countTokens<YouChooseToken>() == 1 &&
               countTokens<YouDoNotChooseToken>() == 1) {
        int globalDirP = takeToken<YouChooseToken>()->origin;
        int localDirP = globalToLocalDir(globalDirP);
        int dir_p = (localDirP + 3) % 6;
        LeaderElectionErosionParticle &p = nbrAtLabel(dir_p);

        int globalDirQ = takeToken<YouDoNotChooseToken>()->origin;
        int localDirQ = globalToLocalDir(globalDirQ);
        int dir_q = (localDirQ + 3) % 6;
        LeaderElectionErosionParticle &q = nbrAtLabel(dir_q);

        int globalizedDirP = localToGlobalDir(dir_p);
        p.putToken(std::make_shared<SameHandednessToken>(globalizedDirP));

        int globalizedDirQ = localToGlobalDir(dir_q);
        q.putToken(std::make_shared<SameHandednessToken>(globalizedDirQ));
      }

      // 4. Leader election phase
      // Participate in the fetching of neighborhood encodings from the
      // subtree.

      // If encoding requested by parent
      if (hasToken<RequestEncodingToken>()) {
        // If not own encoding sent
        if (!nbrhdEncodingSent) {
          // Send neighborhood encoding
          string encoding = getNeighborhoodEncoding();

          sendEncodingParent(encoding);
          nbrhdEncodingSent = true;

          takeToken<RequestEncodingToken>();

          stateStable = true;
          return;
        }
        // If own encoding sent
        else {
          // If not encoding rquested
          if (!sentEncodingRequest) {
            // Request encoding from child
            // Starting from parent, loop through children in clockwise order
            int childDir = (parent + 1) % 6;
            while (children.find(childDir) == children.end() ||
                   childrenExhausted.find(childDir) !=
                       childrenExhausted.end()) {
              childDir = (childDir + 1) % 6;
              if (childDir == parent) {
                break;
              }
            }
            if (childDir == parent) {
              // Tree exhausted
              treeExhausted = true;

              sendExhaustedToken(parent);

              stateStable = true;
              return;
            } else {
              // Request encoding from child
              LeaderElectionErosionParticle &nbr = nbrAtLabel(childDir);
              int globalizedDir = localToGlobalDir(childDir);

              nbr.putToken(
                  std::make_shared<RequestEncodingToken>(globalizedDir));
              sentEncodingRequest = true;

              stateStable = true;
              return;
            }
          }
          // If encoding requested
          else {
            // If encoding received
            if (hasToken<EncodingToken>()) {
              // Take token and forward encoding
              std::shared_ptr<EncodingToken> token =
                  peekAtToken<EncodingToken>();
              string encoding = token->encoding;
              takeToken<EncodingToken>();

              sendEncodingParent(encoding);
              takeToken<RequestEncodingToken>();

              stateStable = true;
              return;
            }
            // If child's subtree is exhausted
            else if (hasToken<SubTreeExhaustedToken>()) {
              // Add child to exhausted list and request encoding from next
              // child
              int globalDir = takeToken<SubTreeExhaustedToken>()->origin;
              int localDir = (globalToLocalDir(globalDir) + 3) % 6;

              childrenExhausted.insert(localDir);

              sentEncodingRequest = false;

              stateStable = true;
              return;
            }
            // If no tokens received yet, wait
            stateStable = true;
            return;
          }
        }
      }
    }
    stateStable = true;
    return;
  } else if (state == State::RootElection) {
    // 4. Leader election phase
    // Query subtrees for neighborhood encodings of particles.
    // Compare these neighborhood encodings to attempt to break symmetry.

    if (!nbrhdEncodingSent) {
      // first step: send own neighborhood encoding
      string encoding = getNeighborhoodEncoding();

      currentEncoding = encoding;

      sendEncodingCandidates(encoding);
      nbrhdEncodingSent = true;
      encodingSent = true;

      stateStable = true;
      return;
    }

    if (sentEncodingRequest) {
      if (hasToken<EncodingToken>()) {
        // Receive encoding token, set encoding and send it
        std::shared_ptr<EncodingToken> token = peekAtToken<EncodingToken>();
        int globalDir = token->origin;
        string encoding = token->encoding;
        takeToken<EncodingToken>();

        currentEncoding = encoding;
        sendEncodingCandidates(encoding);
        encodingSent = true;

        sentEncodingRequest = false;

        stateStable = true;
        return;
      } else if (hasToken<SubTreeExhaustedToken>()) {
        // Child's subtree exhausted, add child to exhausted list
        // and send new encoding request
        int globalDir = takeToken<SubTreeExhaustedToken>()->origin;
        int localDir = (globalToLocalDir(globalDir) + 3) % 6;

        childrenExhausted.insert(localDir);

        sentEncodingRequest = false;

        stateStable = true;
        return;
      }
      stateStable = true;
      return;
    }

    if (encodingSent) {
      // Wait for tokens and then compare

      if (!(countTokens<EncodingTokenCandidate>() == numCandidates)) {
        stateStable = true;
        return;
      }

      if (numCandidates == 1) {
        std::shared_ptr<EncodingTokenCandidate> token =
            takeToken<EncodingTokenCandidate>();
        int globalDir = token->origin;
        string encoding = token->encoding;

        if (currentEncoding < encoding) {
          // Lexicographically smaller -> become leader
          state = State::Leader;
          stateStable = false;
          return;
        } else if (currentEncoding > encoding) {
          // Lexicographically larger -> revoke candidacy
          state = State::Tree;
          parent = (globalToLocalDir(globalDir) + 3) % 6;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        } else {
          // Lexicographically equal -> get next encoding...
          encodingSent = false;
          stateStable = true;
          return;
        }
      } else if (numCandidates == 2) {
        std::shared_ptr<EncodingTokenCandidate> tokenA =
            peekAtToken<EncodingTokenCandidate>();
        int globalDirA = tokenA->origin;
        string encodingA = tokenA->encoding;
        takeToken<EncodingTokenCandidate>();

        std::shared_ptr<EncodingTokenCandidate> tokenB =
            peekAtToken<EncodingTokenCandidate>();
        int globalDirB = tokenB->origin;
        string encodingB = tokenB->encoding;
        takeToken<EncodingTokenCandidate>();

        if (currentEncoding < encodingA && currentEncoding < encodingB) {
          // Lexicographically smallest -> become leader
          state = State::Leader;
          stateStable = false;
          return;
        } else if (encodingA < currentEncoding && encodingA < encodingB) {
          // encoding A is smallest -> A becomes the leader
          state = State::Tree;
          parent = (globalToLocalDir(globalDirA) + 3) % 6;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        } else if (encodingB < currentEncoding && encodingB < encodingA) {
          // encoding B is smallest -> B becomes the leader
          state = State::Tree;
          parent = (globalToLocalDir(globalDirB) + 3) % 6;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        }
        // There is no unique lexicographically smallest encoding
        // Check if there is a unique lexicographically largest encoding...
        else if (currentEncoding > encodingA && currentEncoding > encodingB) {
          // Lexicographically largest -> become leader
          state = State::Leader;
          stateStable = false;
          return;
        } else if (encodingA > currentEncoding && encodingA > encodingB) {
          // encoding A is largest -> A becomes leader
          state = State::Tree;
          parent = (globalToLocalDir(globalDirA) + 3) % 6;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        } else if (encodingB > currentEncoding && encodingB > encodingA) {
          // encoding B is largest -> B becomes leader
          state = State::Tree;
          parent = (globalToLocalDir(globalDirB) + 3) % 6;
          int globalizedDir = localToGlobalDir(parent);
          LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<ParentToken>(globalizedDir));
          stateStable = false;
          return;
        } else {
          // Encodings are equal, move on...
          encodingSent = false;
          stateStable = true;
          return;
        }
      }
      stateStable = true;
      return;
    }

    // If some candidates have not yet processed their tokens, wait.
    for (int dir : candidates) {
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (nbr.countTokens<EncodingTokenCandidate>() == numCandidates) {
        stateStable = true;
        return;
      }
    }

    // Loop through children in direction of increasing labels.
    // Starting from the direction of the adjacent candidate(s).
    int candidateDir = -1;
    for (int c : candidates) {
      candidateDir = c;
    }

    int childDir = (candidateDir + 1) % 6;
    while (children.find(childDir) == children.end() ||
           childrenExhausted.find(childDir) != childrenExhausted.end()) {
      childDir = (childDir + 1) % 6;
      if (childDir == candidateDir) {
        break;
      }
    }

    if (childDir == candidateDir) {
      // Tree exhausted -> unbreakable symmetry by theorem 6
      // Become leader (1 of multiple leaders, algorithm fails)
      state = State::Leader;
      stateStable = false;
      return;
    } else {
      // Request encoding from child
      LeaderElectionErosionParticle &nbr = nbrAtLabel(childDir);
      int globalizedDir = localToGlobalDir(childDir);
      nbr.putToken(std::make_shared<RequestEncodingToken>(globalizedDir));
      sentEncodingRequest = true;
      stateStable = true;
      return;
    }
  } else {
    stateStable = true;
    return;
  }
}

void LeaderElectionErosionParticle::sendExhaustedToken(int dir) {
  LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
  int globalizedDir = localToGlobalDir(dir);
  nbr.putToken(std::make_shared<SubTreeExhaustedToken>(globalizedDir));
}

void LeaderElectionErosionParticle::sendEncodingParent(string encoding) {
  LeaderElectionErosionParticle &nbr = nbrAtLabel(parent);
  int globalizedDir = localToGlobalDir(parent);
  nbr.putToken(std::make_shared<EncodingToken>(globalizedDir, encoding));
}

void LeaderElectionErosionParticle::sendEncodingCandidates(string encoding) {
  for (auto dir : candidates) {
    LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
    int globalizedDir = localToGlobalDir(dir);
    nbr.putToken(
        std::make_shared<EncodingTokenCandidate>(globalizedDir, encoding));
  }
}

string LeaderElectionErosionParticle::getNeighborhoodEncoding() {
  string result = "";
  for (int dir = 0; dir < 6; ++dir) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state == State::Candidate || nbr.state == State::Root ||
          nbr.state == State::RootElection) {
        result = result + "L";
      } else if (dir == parent) {
        result = result + "P";
      } else if (children.find(dir) != children.end()) {
        result = result + "C";
      } else {
        result = result + "N";
      }
    } else {
      result = result + "N";
    }
  }
  return result;
}

int LeaderElectionErosionParticle::getNumCandidates() {
  int num = 0;
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state == State::Candidate || nbr.state == State::Root ||
          nbr.state == State::RootElection) {
        if (nbr.isContracted()) {
          candidates.insert(dir);
        } else {
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
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (children.find(dir) != children.end()) {
        if (!nbr.treeDone) {
          return false;
        }
      } else if (nbr.state != State::Tree && nbr.state != State::Root &&
                 nbr.state != State::RootElection) {
        return false;
      }
    }
  }
  return true;
}

bool LeaderElectionErosionParticle::isLocked() const {
  if (cornerType == 3) {
    // initialize array with integer for each neighbor.
    int nbrs[6] = {};
    for (int dir = 0; dir < 6; dir++) {
      if (hasNbrAtLabel(dir)) {
        LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
        if (nbr.state != State::Eroded) {
          nbrs[dir] = nbr.cornerType;
        } else {
          nbrs[dir] = -3;
        }
      } else {
        nbrs[dir] = -4;
      }
    }
    int eligibleNbrs = 0;
    for (int i = 0; i < 6; i++) {
      eligibleNbrs = eligibleNbrs + nbrs[i];
    }
    if (eligibleNbrs != 3) {
      return false;
    } else if (eligibleNbrs == 3) {
      for (int i = 0; i < 6; i++) {
        if (nbrs[i] >= -2) {
          if (i > 0) {
            if (nbrs[i + 1] >= -2 && nbrs[i + 2] >= -2) {
              return nbrs[i + 1] == 3;
            } else {
              return false;
            }
          } else {
            if (nbrs[i + 1] >= -2 && nbrs[i + 2] >= -2) {
              return nbrs[i + 1] == 3;
            } else if (nbrs[i + 1] >= -2 && nbrs[5] >= -2) {
              return nbrs[i] == 3;
            } else if (nbrs[5] >= -2 && nbrs[4] >= -2) {
              return nbrs[5] == 3;
            } else {
              return false;
            }
          }
        }
      }
    }
  } else {
    return false;
  }
}

void LeaderElectionErosionParticle::updateStability() {
  stable = true;
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (!nbr.stateStable) {
        stable = false;
      }
    }
  }
}

int LeaderElectionErosionParticle::headMarkDir() const {
  if (state == State::Tree) {
    return parent;
  } else {
    return -1;
  }
}

int LeaderElectionErosionParticle::headMarkColor() const {
  if (state == State::Leader) {
    return 0x00ff00;
  } else if (state == State::Candidate) {
    return 0xffaa00;
  } else if (state == State::Eroded) {
    return 0x999999;
  } else if (state == State::Root) {
    if (treeDone) {
      return 0x5a2d00;
    }
    return 0xc46200;
  } else if (state == State::RootElection) {
    return 0xff00ff;
  } else if (state == State::Tree) {
    if (treeDone) {
      if (treeExhausted) {
        return 0x000000;
      } else if (nbrhdEncodingSent) {
        return 0x868686;
      } else {
        return 0x006100;
      }
    }
    return 0x00b000;
  } else if (isLocked()) {
    return 0xfff000;
  }

  return -1;
}

LeaderElectionErosionParticle &
LeaderElectionErosionParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<LeaderElectionErosionParticle>(label);
}

int LeaderElectionErosionParticle::getCornerType() const {
  int nbrs[6] = {};
  for (int dir = 0; dir < 6; dir++) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionErosionParticle &nbr = nbrAtLabel(dir);
      if (nbr.state != State::Eroded) {
        nbrs[dir] = 1;
      } else {
        nbrs[dir] = 0;
      }
    } else {
      nbrs[dir] = 0;
    }
  }

  int eligibleNbrs = 0;
  for (int i = 0; i < 6; i++) {
    eligibleNbrs = eligibleNbrs + nbrs[i];
  }
  if (eligibleNbrs > 3) {
    return -1;
  } else if (eligibleNbrs == 0) {
    return 0;
  } else if (eligibleNbrs == 1) {
    return 1;
  } else if (eligibleNbrs == 2) {
    for (int i = 0; i < 6; i++) {
      if (nbrs[i] == 1) {
        if (i > 0) {
          if (nbrs[i + 1] == 1) {
            return 2;
          } else {
            return -1;
          }
        } else {
          if (nbrs[i + 1] == 1 || nbrs[5] == 1) {
            return 2;
          } else {
            return -1;
          }
        }
      }
    }
  } else if (eligibleNbrs == 3) {
    for (int i = 0; i < 6; i++) {
      if (nbrs[i] == 1) {
        if (i > 0) {
          if (nbrs[i + 1] == 1 && nbrs[i + 2] == 1) {
            return 3;
          } else {
            return -1;
          }
        } else {
          if ((nbrs[i + 1] == 1 && nbrs[i + 2] == 1) ||
              (nbrs[i + 1] == 1 && nbrs[5] == 1) ||
              (nbrs[5] == 1 && nbrs[4] == 1)) {
            return 3;
          } else {
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

QString LeaderElectionErosionParticle::inspectionText() const {
  QString text;
  QString indent = "    ";

  text += "head: (" + QString::number(head.x) + ", " + QString::number(head.y) +
          ")\n";
  text += "orientation: " + QString::number(orientation) + "\n";
  text += "globalTailDir: " + QString::number(globalTailDir) + "\n";
  text += "state: ";
  text += [this]() {
    switch (state) {
    case State::Eligible:
      return "eligible";
    case State::Candidate:
      return "candidate";
    case State::Root:
      return "root";
    case State::Tree:
      return "tree";
    case State::RootElection:
      return "root election";
    case State::Eroded:
      return "eroded";
    case State::Leader:
      return "leader";
    default:
      return "no state";
    }
  }();
  text += "\n";
  text += "stable: " + QString::number(stable) + "\n";
  text += "stateStable: " + QString::number(stateStable) + "\n";
  text += "has leader election tokens: " +
          QString::number(countTokens<LeaderElectionToken>()) + "\n";
  text += "parent: " + QString::number(parent) + "\n";
  text += "children: ";
  for (int child : children) {
    if (!text.endsWith(": ")) {
      text += ", ";
    }
    text += QString::number(child);
  }
  text += "\n";
  text += "numCandidates: " + QString::number(numCandidates) + "\n";
  text += "Candidate encoding tokens: " + QString::number(countTokens<EncodingTokenCandidate>()) + "\n";
  text += "encodingSent: " + QString::number(encodingSent) + "\n";

  return text;
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN SYSTEM CODE----------------------------

#include <string>
#include <fstream>
#include <sstream>
#include <QTextStream>

using namespace std;

LeaderElectionErosionSystem::LeaderElectionErosionSystem(int numParticles, QString fileName) {
  Q_ASSERT(numParticles > 0 || fileName.size() > 0);

  randomPermutationScheduler = true;

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

      insert(new LeaderElectionErosionParticle(
      Node(x, y), -1, randDir(), *this,
      LeaderElectionErosionParticle::State::Eligible));
    }

    file.close();

    outputPath = "../AmoebotSim/data/output/" + fileName.toStdString() + ".txt";

    out << "Particle system initialized from file." << endl;
    
    return;
  }

  // Insert the seed at (0,0).
  insert(new LeaderElectionErosionParticle(
      Node(0, 0), -1, randDir(), *this,
      LeaderElectionErosionParticle::State::Eligible));
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
          insert(new LeaderElectionErosionParticle(
              nbr, -1, randDir(), *this,
              LeaderElectionErosionParticle::State::Eligible));
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
    auto hp = dynamic_cast<LeaderElectionErosionParticle *>(p);
    if (hp->state == LeaderElectionErosionParticle::State::Leader) {
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