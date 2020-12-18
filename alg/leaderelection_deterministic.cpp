/* Copyright (C) 2020 Remco Surtel.
 * The full GNU GPLv3 can be found in the LICENSE file, and the full copyright
 * notice can be found at the top of main/main.cpp.
 *
 * Deterministic leader election.
 * Based on the paper "Deterministic Leader Election in Programmable Matter"
 * By Yuval Emek, Shay Kutten, Ron Lavi, and William K. Moses Jr.
 * https://arxiv.org/abs/1905.00580 */

#include "alg/leaderelection_deterministic.h"

#include <set>

#include <QtGlobal>

#include <QDebug>

using namespace std;

//----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionDeterministicParticle::LeaderElectionDeterministicParticle(
    const Node head, const int globalTailDir, const int orientation,
    AmoebotSystem &system, State state)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state) {}

void LeaderElectionDeterministicParticle::activate() {
  if (state == State::Initlialization) {
    if (!isBoundaryParticle()) {
      // Particles not on a boundary change to phase 2 and wait
      state = State::ForestFormation;
      return;
    }
    else {
      // Initialize variables
      if (labels.size() == 0) {
        setLabels();
        for (int i = 0; i < numBoundaries(); i++) {
          segHeads.push_back(true);
          counts.push_back(labels.at(i));
          successors.push_back(-1);
          predecessors.push_back(-1);
          countsRequested.push_back(false);
          mergesRequested.push_back(false);
          lexicoGraphicComparisons.push_back(false);
          sentLabels.push_back(false);
          sentNbrLabels.push_back(false);
          reqLabels.push_back(false);
          reqNbrLabels.push_back(false);
          reqLabelsForNbr.push_back(false);
          receivedLabels.push_back(false);
          receivedNbrLabels.push_back(false);
          internalLabels.push_back(0);
          nbrLabels.push_back(0);
          endOfSegments.push_back(false);
          endOfNbrSegments.push_back(false);
          firstLargerLabels.push_back(0);
          terminationDetections.push_back(false);
        }
      }

      // Execute the following code for each boundary
      for (int i = 0; i < numBoundaries(); i++) {
        // Variables for this boundary
        bool segHead = segHeads[i];
        int label = labels[i];
        int count = counts[i];
        int successor = successors[i];
        int predecessor = predecessors[i];
        int nextNbr = nextDir(i);
        int prevNbr = prevDir(i);
        bool countRequested = countsRequested[i];
        bool mergeRequested = mergesRequested[i];
        bool lexicoGraphicComparison = lexicoGraphicComparisons[i];
        bool sentLabel = sentLabels[i];
        bool sentNbrLabel = sentNbrLabels[i];
        bool reqLabel = reqLabels[i];
        bool reqLabelForNbr = reqLabelsForNbr[i];
        bool reqNbrLabel = reqNbrLabels[i];
        bool receivedLabel = receivedLabels[i];
        bool receivedNbrLabel = receivedNbrLabels[i];
        int internalLabel = internalLabels[i];
        int nbrLabel = nbrLabels[i];
        bool endOfSegment = endOfSegments[i];
        bool endOfNbrSegment = endOfNbrSegments[i];
        int firstLargerLabel = firstLargerLabels[i];
        bool terminationDetection = terminationDetections[i];

        if (segHead) {
          qDebug() << "Seg_head particle: " << QString::number(head.x) << ", " << QString::number(head.y) << "; Label = " << QString::number(label);
          // Head (and possibly also tail) of segment

          if (hasToken<TerminationToken>()) {
            std::shared_ptr<TerminationToken> token = peekAtToken<TerminationToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<TerminationToken>();
              if (token->traversed + 1 < token->ttl) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<TerminationToken>(localToGlobalDir((prevNbr+3)%6), token->ttl, token->traversed+1));
              }
              numCandidates = token->ttl;
              state = State::ForestFormationCandidate;
              return;
            }
          }

          // receive and process count request tokens
          if (hasToken<CountRequestToken>()) {
            std::shared_ptr<CountRequestToken> token = peekAtToken<CountRequestToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              qDebug() << "Sending count: " << QString::number(count);
              takeToken<CountRequestToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<CountToken>(localToGlobalDir((prevNbr+3)%6), count));
            }
          }

          // receive and process termination detection tokens
          if (hasToken<TerminationDetectionToken>()) {
            std::shared_ptr<TerminationDetectionToken> token = peekAtToken<TerminationDetectionToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              if (token->count != count || mergeRequested) {
                takeToken<TerminationDetectionToken>();
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->traversed+1, 1, false));
              }
              else {
                // counts equal -> await lexicographic comparison
                if (!lexicoGraphicComparison) {
                  cleanup(i);
                  lexicoGraphicComparisons[i] = true;
                  lexicoGraphicComparison = true;
                  qDebug() << "Starting lexicographic comparison...";
                }
              }
            }
          }
          if (hasToken<TerminationDetectionReturnToken>()) {
            std::shared_ptr<TerminationDetectionReturnToken> token = peekAtToken<TerminationDetectionReturnToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<TerminationDetectionReturnToken>();
              if (token->traversed + 1 == token->ttl) {
                terminationDetection = false;
                terminationDetections[i] = false;
                if (token->termination) {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                  nbr.putToken(std::make_shared<TerminationToken>(localToGlobalDir((prevNbr+3)%6), 6/count, 1));
                  numCandidates = 6/count;
                  state = State::ForestFormationCandidate;
                  return;
                }
              }
              else if (token->count == count) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->ttl, token->traversed+1, token->termination));
              }
              else {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->ttl, token->traversed+1, false));
              }
            }
          }

          // receive and process merge request tokens
          if (hasToken<MergeRequestCountToken>()) {
            std::shared_ptr<MergeRequestCountToken> token = peekAtToken<MergeRequestCountToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<MergeRequestCountToken>();
              qDebug() << "Received merge request...";
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              if (mergeRequested || !(token->count > count && token->count + count <= 6 && token->count > 0)) {
                nbr.putToken(std::make_shared<MergeNackToken>(localToGlobalDir((prevNbr+3)%6)));
                qDebug() << "Merge declined";
              }
              else {
                qDebug() << "Acknowledging merge...";
                nbr.putToken(std::make_shared<MergeAckToken>(localToGlobalDir((prevNbr+3)%6), count));
                predecessors[i] = prevNbr;
                segHeads[i] = false;
                // interrupt lexicographic comparison with next segment if applicable
                if (lexicoGraphicComparison) {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                  nbr.putToken(std::make_shared<LexCompInterruptNextToken>(localToGlobalDir((nextNbr+3)%6)));
                  cleanup(i);
                }
                continue;
              }
            }
          }
          if (hasToken<LexCompMergeRequestToken>()) {
            std::shared_ptr<LexCompMergeRequestToken> token = peekAtToken<LexCompMergeRequestToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompMergeRequestToken>();
              qDebug() << "Received lexicographic comparison merge request...";
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              if (mergeRequested || !(token->count == count && token->count + count <= 6 && token->count > 0)) {
                nbr.putToken(std::make_shared<MergeNackToken>(localToGlobalDir((prevNbr+3)%6)));
                qDebug() << "Merge declined";
              }
              else {
                qDebug() << "Acknowledging merge...";
                nbr.putToken(std::make_shared<MergeAckToken>(localToGlobalDir((prevNbr+3)%6), count));
                predecessors[i] = prevNbr;
                segHeads[i] = false;
                // interrupt lexicographic comparison with next segment if applicable
                if (lexicoGraphicComparison) {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                  nbr.putToken(std::make_shared<LexCompInterruptNextToken>(localToGlobalDir((nextNbr+3)%6)));
                  cleanup(i);
                }
                continue;
              }
            }
          }

          // receive and process interrupt tokens
          if (hasToken<LexCompInterruptNextToken>()) {
            std::shared_ptr<LexCompInterruptNextToken> token = peekAtToken<LexCompInterruptNextToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompInterruptNextToken>();
              cleanupForNbr(i);
              continue;
            }
          }
          if (hasToken<LexCompInterruptPrevToken>()) {
            std::shared_ptr<LexCompInterruptPrevToken> token = peekAtToken<LexCompInterruptPrevToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompInterruptPrevToken>();
              cleanup(i);
              continue;
            }
          }

          // receive label requests from previous segment
          if (hasToken<LexCompRequestNbrLabelToken>()) {
            std::shared_ptr<LexCompRequestNbrLabelToken> token = peekAtToken<LexCompRequestNbrLabelToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompRequestNbrLabelToken>();
              if (!sentNbrLabel) {
                qDebug() << "Sending label to neighbor: " << QString::number(label);
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrLabelToken>(localToGlobalDir((prevNbr+3)%6), label));
                sentNbrLabels[i] = true;
                sentNbrLabel = true;
              }
              else if (successor == -1) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrEndOfSegmentToken>(localToGlobalDir((prevNbr+3)%6)));
                cleanupForNbr(i);
              }
              else {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompReqLabelForNbrToken>(localToGlobalDir((nextNbr+3)%6)));
                reqLabelsForNbr[i] = true;
                reqLabelForNbr = true;
              }
            }
          }

          if (reqLabelForNbr) {
            // receive labels for previous segment and send them
            if (hasToken<LexCompReturnLabelForNbrToken>()) {
              std::shared_ptr<LexCompReturnLabelForNbrToken> token = peekAtToken<LexCompReturnLabelForNbrToken>();
              if (globalToLocalDir(token->origin) == nextNbr) {
                takeToken<LexCompReturnLabelForNbrToken>();
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrLabelToken>(localToGlobalDir((prevNbr+3)%6), token->label));
                reqLabelForNbr = false;
                reqLabelsForNbr[i] = false;
              }
            }
            if (hasToken<LexCompEndOfSegmentForNbrToken>()) {
              // receive end of segment tokens and send them to previous segment
              std::shared_ptr<LexCompEndOfSegmentForNbrToken> token = peekAtToken<LexCompEndOfSegmentForNbrToken>();
              if (globalToLocalDir(token->origin) == nextNbr) {
                takeToken<LexCompEndOfSegmentForNbrToken>();
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrEndOfSegmentToken>(localToGlobalDir((prevNbr+3)%6)));
                reqLabelForNbr = false;
                reqLabelsForNbr[i] = false;
                cleanupForNbr(i);
              }
            }
          }

          // Lexicographic comparison
          if (lexicoGraphicComparison) {
            if (!reqLabel && !receivedLabel) {
              // request internal labels
              if (!sentLabel) {
                qDebug() << "Sending label: " << QString::number(label);
                internalLabel = label;
                internalLabels[i] = label;
                sentLabels[i] = true;
                sentLabel = true;
                receivedLabels[i] = true;
                receivedLabel = true;
              }
              else if (successor == -1) {
                endOfSegments[i] = true;
                endOfSegment = true;
                receivedLabels[i] = true;
                receivedLabel = true;
              }
              else {
                qDebug() << "Requesting internal label...";
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompReqLabelToken>(localToGlobalDir((nextNbr+3)%6)));
                reqLabels[i] = true;
                reqLabel = true;
              }
            }
            if (reqLabel && !receivedLabel) {
              // receive internal labels
              if (hasToken<LexCompReturnLabelToken>()) {
                std::shared_ptr<LexCompReturnLabelToken> token = peekAtToken<LexCompReturnLabelToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  qDebug() << "Receiving internal label: " << QString::number(token->label);
                  takeToken<LexCompReturnLabelToken>();
                  internalLabel = token->label;
                  internalLabels[i] = token->label;
                  reqLabel = false;
                  reqLabels[i] = false;
                  receivedLabel = true;
                  receivedLabels[i] = true;
                }
              }
              // receive end of segment tokens
              if (hasToken<LexCompEndOfSegmentToken>()) {
                std::shared_ptr<LexCompEndOfSegmentToken> token = peekAtToken<LexCompEndOfSegmentToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  qDebug() << "Receiving internal end of segment token...";
                  takeToken<LexCompEndOfSegmentToken>();
                  reqLabel = false;
                  reqLabels[i] = false;
                  receivedLabel = true;
                  receivedLabels[i] = true;
                  endOfSegment = true;
                  endOfSegments[i] = true;
                }
              }
            }
            if (!reqNbrLabel && !receivedNbrLabel) {
              // request labels from next segment
              qDebug() << "Requesting label from next segment...";
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<LexCompRequestNbrLabelToken>(localToGlobalDir((nextNbr+3)%6)));
              reqNbrLabels[i] = true;
              reqNbrLabel = true;
            }
            if (hasToken<LexCompReturnNbrLabelToken>() && reqNbrLabel && !receivedNbrLabel) {
              // receive labels from next segment
              std::shared_ptr<LexCompReturnNbrLabelToken> token = peekAtToken<LexCompReturnNbrLabelToken>();
              if (globalToLocalDir(token->origin) == nextNbr) {
                qDebug() << "Receiving label from next segment: " << QString::number(token->label);
                takeToken<LexCompReturnNbrLabelToken>();
                nbrLabel = token->label;
                nbrLabels[i] = token->label;
                reqNbrLabel = false;
                reqNbrLabels[i] = false;
                receivedNbrLabel = true;
                receivedNbrLabels[i] = true;
              }
            }
            if (hasToken<LexCompReturnNbrEndOfSegmentToken>() && reqNbrLabel && !receivedNbrLabel) {
              // receive end of segment tokens from next segment
              std::shared_ptr<LexCompReturnNbrEndOfSegmentToken> token = peekAtToken<LexCompReturnNbrEndOfSegmentToken>();
              if (globalToLocalDir(token->origin) == nextNbr) {
                qDebug() << "Receiving end of segment token from next segment...";
                takeToken<LexCompReturnNbrEndOfSegmentToken>();
                endOfNbrSegment = true;
                endOfNbrSegments[i] = true;
                reqNbrLabel = false;
                reqNbrLabels[i] = false;
                receivedNbrLabel = true;
                receivedNbrLabels[i] = true;
              }
            }
            if (receivedLabel && receivedNbrLabel) {
              qDebug() << "Received 2 labels, comparing: " << QString::number(internalLabel) << ", " << QString::number(nbrLabel);
              // compare received labels
              if (firstLargerLabel == 0 && !endOfSegment && !endOfNbrSegment) {
                if (internalLabel > nbrLabel) {
                  firstLargerLabel = 1;
                  firstLargerLabels[i] = 1;
                }
                else if (nbrLabel > internalLabel) {
                  firstLargerLabel = -1;
                  firstLargerLabels[i] = -1;
                }
              }
              if (firstLargerLabel != 0) {
                // if not lexicographically equal, send back any termination detection tokens
                if (hasToken<TerminationDetectionToken>()) {
                  std::shared_ptr<TerminationDetectionToken> token = peekAtToken<TerminationDetectionToken>();
                  if (globalToLocalDir(token->origin) == nextNbr) {
                    takeToken<TerminationDetectionToken>();
                    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                    nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->traversed+1, 1, false));
                  }
                }
              }
              if (!endOfSegment && !endOfNbrSegment) {
                // received 2 labels, request next
                receivedLabel = false;
                receivedLabels[i] = false;
                receivedNbrLabel = false;
                receivedNbrLabels[i] = false;
              }
              else if (endOfSegment && !endOfNbrSegment) {
                // finish comparison (loss), interrupt next segment, cleanup
                qDebug() << "Lexicographically smaller";
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompInterruptNextToken>(localToGlobalDir((nextNbr+3)%6)));
                cleanup(i);
                continue;
              }
              else if (!endOfSegment && endOfNbrSegment) {
                // finish comparison (win), request merge, cleanup
                qDebug() << "Lexicographically larger";
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompMergeRequestToken>(localToGlobalDir((nextNbr+3)%6), count));
                mergeRequested = true;
                mergesRequested[i] = true;
                cleanup(i);
                // interrupt lexicographic comparison with previous segment if applicable
                if (sentNbrLabel) {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                  nbr.putToken(std::make_shared<LexCompInterruptPrevToken>(localToGlobalDir((prevNbr+3)%6)));
                  cleanupForNbr(i);
                }
                continue;
              }
              else if (endOfSegment && endOfNbrSegment) {
                // finish comparison (use firstLargerLabel), cleanup
                if (firstLargerLabel == 1) {
                  // win, request merge, cleanup
                  qDebug() << "Lexicographically larger";
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                  nbr.putToken(std::make_shared<LexCompMergeRequestToken>(localToGlobalDir((nextNbr+3)%6), count));
                  mergeRequested = true;
                  mergesRequested[i] = true;
                  cleanup(i);
                  // interrupt lexicographic comparison with previous segment if applicable
                  if (sentNbrLabel) {
                    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                    nbr.putToken(std::make_shared<LexCompInterruptPrevToken>(localToGlobalDir((prevNbr+3)%6)));
                    cleanupForNbr(i);
                  }
                  continue;
                }
                else if (firstLargerLabel == -1) {
                  // loss, cleanup
                  qDebug() << "Lexicographically smaller";
                  cleanup(i);
                  continue;
                }
                // if lexicographically equal -> termination detection
                else if (firstLargerLabel == 0) {
                  // termination detection
                  qDebug() << "Lexicographically equal: termination detection... Count: " << QString::number(count);
                  if ((count == 1 || count == 2 || count == 3) && !terminationDetection) {
                    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                    nbr.putToken(std::make_shared<TerminationDetectionToken>(localToGlobalDir((prevNbr+3)%6), count, 6/count+1, 1));
                    terminationDetection = true;
                    terminationDetections[i] = true;
                  }
                  else if (count == 6) {
                    qDebug() << "Lexicographically equal with count 6 -> terminating...";
                    state = State::Leader;
                    return;
                  }

                  if (hasToken<TerminationDetectionToken>()) {
                    std::shared_ptr<TerminationDetectionToken> token = peekAtToken<TerminationDetectionToken>();
                    if (globalToLocalDir(token->origin) == nextNbr) {
                      takeToken<TerminationDetectionToken>();
                      if (token->traversed + 1 == token->ttl) {
                        LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                        nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), count, 6/count+1, 1, true));
                      }
                      else {
                        LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                        nbr.putToken(std::make_shared<TerminationDetectionToken>(localToGlobalDir((prevNbr+3)%6), count, token->ttl, token->traversed+1));
                      }
                    }
                  }
                }
              }
            }
          }

          // If positive count, try to merge with next segment
          else if (count > 0) {
            if (!countRequested && !mergeRequested) {
              // Request count of next segment to determine if they should merge
              qDebug() << "Requesting count... Count: " << QString::number(count);
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<CountRequestToken>(localToGlobalDir((nextNbr+3)%6)));
              countsRequested[i] = true;
            }
            else if (!mergeRequested) {
              if (hasToken<CountToken>()) {
                std::shared_ptr<CountToken> token = peekAtToken<CountToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  takeToken<CountToken>();
                  countsRequested[i] = false;
                  if (count > token->count && count + token->count <= 6) {
                    // Merge
                    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                    nbr.putToken(std::make_shared<MergeRequestCountToken>(localToGlobalDir((nextNbr+3)%6), count));
                    mergesRequested[i] = true;
                    mergeRequested = true;
                    // interrupt lexicographic comparison with previous segment if applicable
                    if (sentNbrLabel) {
                      LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                      nbr.putToken(std::make_shared<LexCompInterruptPrevToken>(localToGlobalDir((prevNbr+3)%6)));
                      cleanupForNbr(i);
                    }
                  }
                  else if (count == token->count && (count == 1 || count == 2 || count == 3 || count == 6)) {
                    // Start lexicographic comparison
                    if (!lexicoGraphicComparison) {
                      cleanup(i);
                      lexicoGraphicComparisons[i] = true;
                      lexicoGraphicComparison = true;
                      qDebug() << "Starting lexicographic comparison...";
                    }
                  }
                }
              }
            }
            else {
              if (hasToken<MergeAckToken>()) {
                std::shared_ptr<MergeAckToken> token = peekAtToken<MergeAckToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  takeToken<MergeAckToken>();
                  count += token->count;
                  counts[i] = count;
                  if (successor == -1) {
                    successors[i] = nextNbr;
                    successor = nextNbr;
                  }
                  mergesRequested[i] = false;
                  mergeRequested = false;
                }
              }
              if (hasToken<MergeNackToken>()) {
                std::shared_ptr<MergeNackToken> token = peekAtToken<MergeNackToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  takeToken<MergeNackToken>();
                  mergesRequested[i] = false;
                  mergeRequested = false;
                }
              }
            }
          }
        }
        else {
          // Tail or internal node of segment consisting of multiple particles
          if (hasToken<TerminationToken>()) {
            std::shared_ptr<TerminationToken> token = peekAtToken<TerminationToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<TerminationToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<TerminationToken>(localToGlobalDir((prevNbr+3)%6), token->ttl, token->traversed));
            }
          }

          if (hasToken<CountRequestToken>()) {
            std::shared_ptr<CountRequestToken> token = peekAtToken<CountRequestToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<CountRequestToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<CountRequestToken>(localToGlobalDir((nextNbr+3)%6)));
            }
          }
          if (hasToken<CountToken>()) {
            std::shared_ptr<CountToken> token = peekAtToken<CountToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<CountToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<CountToken>(localToGlobalDir((prevNbr+3)%6), token->count));
            }
          }
          if (hasToken<MergeRequestCountToken>()) {
            std::shared_ptr<MergeRequestCountToken> token = peekAtToken<MergeRequestCountToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<MergeRequestCountToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<MergeRequestCountToken>(localToGlobalDir((nextNbr+3)%6), token->count));
              if (hasToken<TerminationDetectionToken>()) {
                std::shared_ptr<TerminationDetectionToken> token = peekAtToken<TerminationDetectionToken>();
                if (globalToLocalDir(token->origin) == nextNbr) {
                  takeToken<TerminationDetectionToken>();
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                  nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->traversed, 0, false));
                }
              }
            }
          }
          if (hasToken<MergeAckToken>()) {
            std::shared_ptr<MergeAckToken> token = peekAtToken<MergeAckToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<MergeAckToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<MergeAckToken>(localToGlobalDir((prevNbr+3)%6), token->count));
              if (successor == -1) {
                successors[i] = nextNbr;
                successor = nextNbr;
              }
            }
          }
          if (hasToken<MergeNackToken>()) {
            std::shared_ptr<MergeNackToken> token = peekAtToken<MergeNackToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<MergeNackToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<MergeNackToken>(localToGlobalDir((prevNbr+3)%6)));
            }
          }

          // process and/or pass lexicographic comparison tokens
          bool interruptedNext = false;
          if (hasToken<LexCompCleanupToken>()) {
            std::shared_ptr<LexCompCleanupToken> token = peekAtToken<LexCompCleanupToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompCleanupToken>();
              cleanup(i);
              continue;
            }
          }
          if (hasToken<LexCompCleanupForNbrToken>()) {
            std::shared_ptr<LexCompCleanupForNbrToken> token = peekAtToken<LexCompCleanupForNbrToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompCleanupForNbrToken>();
              cleanupForNbr(i);
              continue;
            }
          }
          if (hasToken<LexCompInterruptNextToken>()) {
            std::shared_ptr<LexCompInterruptNextToken> token = peekAtToken<LexCompInterruptNextToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompInterruptNextToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<LexCompInterruptNextToken>(localToGlobalDir((nextNbr+3)%6)));
            }
          }
          if (hasToken<LexCompInterruptPrevToken>()) {
            std::shared_ptr<LexCompInterruptPrevToken> token = peekAtToken<LexCompInterruptPrevToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompInterruptPrevToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<LexCompInterruptPrevToken>(localToGlobalDir((prevNbr+3)%6)));
              interruptedNext = true;
            }
          }
          if (hasToken<LexCompRequestNbrLabelToken>()) {
            std::shared_ptr<LexCompRequestNbrLabelToken> token = peekAtToken<LexCompRequestNbrLabelToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompRequestNbrLabelToken>();
              if (!interruptedNext) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompRequestNbrLabelToken>(localToGlobalDir((nextNbr+3)%6)));
              }
            }
          }
          if (hasToken<LexCompReturnNbrLabelToken>()) {
            std::shared_ptr<LexCompReturnNbrLabelToken> token = peekAtToken<LexCompReturnNbrLabelToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompReturnNbrLabelToken>();
              if (!interruptedNext) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrLabelToken>(localToGlobalDir((prevNbr+3)%6), token->label));
              }
            }
          }
          if (hasToken<LexCompReturnNbrEndOfSegmentToken>()) {
            std::shared_ptr<LexCompReturnNbrEndOfSegmentToken> token = peekAtToken<LexCompReturnNbrEndOfSegmentToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompReturnNbrEndOfSegmentToken>();
              if (!interruptedNext) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnNbrEndOfSegmentToken>(localToGlobalDir((prevNbr+3)%6)));
              }
            }
          }
          if (hasToken<LexCompReqLabelToken>()) {
            std::shared_ptr<LexCompReqLabelToken> token = peekAtToken<LexCompReqLabelToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompReqLabelToken>();
              if (!interruptedNext) {
                if (!sentLabel) {
                  qDebug() << "Sending internal label: " << QString::number(label);
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                  nbr.putToken(std::make_shared<LexCompReturnLabelToken>(localToGlobalDir((prevNbr+3)%6), label));
                  sentLabels[i] = true;
                  sentLabel = true;
                }
                else if (successor != -1) {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                  nbr.putToken(std::make_shared<LexCompReqLabelToken>(localToGlobalDir((nextNbr+3)%6)));
                }
                else {
                  LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                  nbr.putToken(std::make_shared<LexCompEndOfSegmentToken>(localToGlobalDir((prevNbr+3)%6)));
                }
              }
            }
          }
          if (hasToken<LexCompReqLabelForNbrToken>()) {
            std::shared_ptr<LexCompReqLabelForNbrToken> token = peekAtToken<LexCompReqLabelForNbrToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompReqLabelForNbrToken>();
              if (!sentNbrLabel) {
                qDebug() << "Sending label to neighbor: " << QString::number(label);
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnLabelForNbrToken>(localToGlobalDir((prevNbr+3)%6), label));
                sentNbrLabels[i] = true;
                sentNbrLabel = true;
              }
              else if (successor != -1) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
                nbr.putToken(std::make_shared<LexCompReqLabelForNbrToken>(localToGlobalDir((nextNbr+3)%6)));
              }
              else {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompEndOfSegmentForNbrToken>(localToGlobalDir((prevNbr+3)%6)));
              }
            }
          }
          if (hasToken<LexCompReturnLabelToken>()) {
            std::shared_ptr<LexCompReturnLabelToken> token = peekAtToken<LexCompReturnLabelToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompReturnLabelToken>();
              if (!interruptedNext) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompReturnLabelToken>(localToGlobalDir((prevNbr+3)%6), token->label));
              }
            }
          }
          if (hasToken<LexCompEndOfSegmentToken>()) {
            std::shared_ptr<LexCompEndOfSegmentToken> token = peekAtToken<LexCompEndOfSegmentToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompEndOfSegmentToken>();
              if (!interruptedNext) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
                nbr.putToken(std::make_shared<LexCompEndOfSegmentToken>(localToGlobalDir((prevNbr+3)%6)));
              }
            }
          }
          if (hasToken<LexCompReturnLabelForNbrToken>()) {
            std::shared_ptr<LexCompReturnLabelForNbrToken> token = peekAtToken<LexCompReturnLabelForNbrToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompReturnLabelForNbrToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<LexCompReturnLabelForNbrToken>(localToGlobalDir((prevNbr+3)%6), token->label));
            }
          }
          if (hasToken<LexCompEndOfSegmentForNbrToken>()) {
            std::shared_ptr<LexCompEndOfSegmentForNbrToken> token = peekAtToken<LexCompEndOfSegmentForNbrToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<LexCompEndOfSegmentForNbrToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<LexCompEndOfSegmentForNbrToken>(localToGlobalDir((prevNbr+3)%6)));
            }
          }
          if (hasToken<LexCompMergeRequestToken>()) {
            std::shared_ptr<LexCompMergeRequestToken> token = peekAtToken<LexCompMergeRequestToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<LexCompMergeRequestToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<LexCompMergeRequestToken>(localToGlobalDir((nextNbr+3)%6), token->count));
            }
          }

          // pass termination detection tokens
          if (hasToken<TerminationDetectionToken>()) {
            std::shared_ptr<TerminationDetectionToken> token = peekAtToken<TerminationDetectionToken>();
            if (globalToLocalDir(token->origin) == nextNbr) {
              takeToken<TerminationDetectionToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(prevNbr);
              nbr.putToken(std::make_shared<TerminationDetectionToken>(localToGlobalDir((prevNbr+3)%6), token->count, token->ttl, token->traversed));
            }
          }
          if (hasToken<TerminationDetectionReturnToken>()) {
            std::shared_ptr<TerminationDetectionReturnToken> token = peekAtToken<TerminationDetectionReturnToken>();
            if (globalToLocalDir(token->origin) == prevNbr) {
              takeToken<TerminationDetectionReturnToken>();
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
              nbr.putToken(std::make_shared<TerminationDetectionReturnToken>(localToGlobalDir((nextNbr+3)%6), token->count, token->ttl, token->traversed, token->termination));
            }
          }
        }
      }
    }
  }
  else if (state == State::ForestFormationCandidate) {
    if (!inTree) {
      while (hasToken<TreeJoinRequestToken>()) {
        std::shared_ptr<TreeJoinRequestToken> token = takeToken<TreeJoinRequestToken>();
        int reqDir = globalToLocalDir(token->origin);
        LeaderElectionDeterministicParticle &nbr = nbrAtLabel(reqDir);
        nbr.putToken(std::make_shared<JoinTreeNackToken>(localToGlobalDir((reqDir+3)%6)));
        requestedTreeJoin.insert(reqDir);
      }
      inTree = true;
      for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
          if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
            LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
            nbr.putToken(std::make_shared<TreeJoinRequestToken>(localToGlobalDir((dir+3)%6)));
          }
        }
      }
    }
    else if (!treeDone) {
      while (hasToken<JoinTreeAckToken>()) {
        std::shared_ptr<JoinTreeAckToken> token = takeToken<JoinTreeAckToken>();
        int childDir = globalToLocalDir(token->origin);
        children.insert(childDir);
      }
      while (hasToken<JoinTreeNackToken>()) {
        std::shared_ptr<JoinTreeNackToken> token = takeToken<JoinTreeNackToken>();
        int nackDir = globalToLocalDir(token->origin);
        nackReceived.insert(nackDir);
      }
      bool done = true;
      for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
          if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
            if (children.find(dir) == children.end() && nackReceived.find(dir) == nackReceived.end()) {
              done = false;
              break;
            }
          }
        }
      }
      if (done) {
        int dir = nextDir(0);
        LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
        nbr.putToken(std::make_shared<CandidateTreeDoneToken>(localToGlobalDir((dir+3)%6), numCandidates+1, 1));
        treeDone = true;
      }
    }
    else if (candidateTreesDone < numCandidates) {
      while (hasToken<CandidateTreeDoneToken>()) {
        std::shared_ptr<CandidateTreeDoneToken> token = takeToken<CandidateTreeDoneToken>();
        candidateTreesDone += 1;
        if (token->traversed + 1 < token->ttl) {
          int dir = nextDir(0);
          LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
          nbr.putToken(std::make_shared<CandidateTreeDoneToken>(localToGlobalDir((dir+3)%6), token->ttl, token->traversed+1));
        }
      }
    }
    else {
      for (int dir : children) {
        LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
        nbr.putToken(std::make_shared<ForestDoneToken>(localToGlobalDir((dir+3)%6)));
      }
      state = State::Candidate;
    }
  }
  else if (state == State::ForestFormation) {
    if (numBoundaries() == 0) {
      if (!inTree) {
        while (hasToken<TreeJoinRequestToken>()) {
          std::shared_ptr<TreeJoinRequestToken> token = takeToken<TreeJoinRequestToken>();
          int reqDir = globalToLocalDir(token->origin);
          requestedTreeJoin.insert(reqDir);
        }
        if (requestedTreeJoin.size() > 0) {
          for (int dir = 0; dir < 6; dir++) {
            if (hasNbrAtLabel(dir)) {
              if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
                nbr.putToken(std::make_shared<TreeJoinRequestToken>(localToGlobalDir((dir+3)%6)));
              }
            }
          }
          inTree = true;
        }
      }
      else if (!treeDone) {
        while (hasToken<JoinTreeAckToken>()) {
          std::shared_ptr<JoinTreeAckToken> token = takeToken<JoinTreeAckToken>();
          int childDir = globalToLocalDir(token->origin);
          children.insert(childDir);
        }
        while (hasToken<JoinTreeNackToken>()) {
          std::shared_ptr<JoinTreeNackToken> token = takeToken<JoinTreeNackToken>();
          int nackDir = globalToLocalDir(token->origin);
          nackReceived.insert(nackDir);
        }
        bool done = true;
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
              if (children.find(dir) == children.end() && nackReceived.find(dir) == nackReceived.end()) {
                done = false;
                break;
              }
            }
          }
        }
        if (done) {
          for (int dir = 0; dir < 6; dir++) {
            if (requestedTreeJoin.find(dir) != requestedTreeJoin.end()) {
              LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
              if (parent == -1) {
                parent = dir;
                nbr.putToken(std::make_shared<JoinTreeAckToken>(localToGlobalDir((dir+3)%6)));
              }
              else {
                nbr.putToken(std::make_shared<JoinTreeNackToken>(localToGlobalDir((dir+3)%6)));
              }
            }
          }
          treeDone = true;
        }
      }
      else if (hasToken<ForestDoneToken>()) {
        takeToken<ForestDoneToken>();
        for (int dir : children) {
          LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
          nbr.putToken(std::make_shared<ForestDoneToken>(localToGlobalDir((dir+3)%6)));
        }
        state = State::Convexification;
      }
    }
    else {
      if (!inTree) {
        while (hasToken<TreeJoinRequestToken>()) {
          std::shared_ptr<TreeJoinRequestToken> token = takeToken<TreeJoinRequestToken>();
          int reqDir = globalToLocalDir(token->origin);
          LeaderElectionDeterministicParticle &nbr = nbrAtLabel(reqDir);
          if (reqDir != prevDir(0)) {
            nbr.putToken(std::make_shared<JoinTreeNackToken>(localToGlobalDir((reqDir+3)%6)));
          }
          requestedTreeJoin.insert(reqDir);
        }
        inTree = true;
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
              if (dir != prevDir(0)) {
                LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
                nbr.putToken(std::make_shared<TreeJoinRequestToken>(localToGlobalDir((dir+3)%6)));
              }
            }
          }
        }
      }
      else if (!treeDone) {
        while (hasToken<TreeJoinRequestToken>()) {
          takeToken<TreeJoinRequestToken>();
        }
        while (hasToken<JoinTreeAckToken>()) {
          std::shared_ptr<JoinTreeAckToken> token = takeToken<JoinTreeAckToken>();
          int childDir = globalToLocalDir(token->origin);
          children.insert(childDir);
        }
        while (hasToken<JoinTreeNackToken>()) {
          std::shared_ptr<JoinTreeNackToken> token = takeToken<JoinTreeNackToken>();
          int nackDir = globalToLocalDir(token->origin);
          nackReceived.insert(nackDir);
        }
        bool done = true;
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            if (requestedTreeJoin.find(dir) == requestedTreeJoin.end()) {
              if (children.find(dir) == children.end() && nackReceived.find(dir) == nackReceived.end() && dir != prevDir(0)) {
                done = false;
                break;
              }
            }
          }
        }
        if (done) {
          treeDone = true;
          parent = prevDir(0);
          LeaderElectionDeterministicParticle &nbr = nbrAtLabel(parent);
          nbr.putToken(std::make_shared<JoinTreeAckToken>(localToGlobalDir((parent+3)%6)));
        }
      }
      else {
        while (hasToken<TreeJoinRequestToken>()) {
          takeToken<TreeJoinRequestToken>();
        }
        while (hasToken<CandidateTreeDoneToken>()) {
          std::shared_ptr<CandidateTreeDoneToken> token = takeToken<CandidateTreeDoneToken>();
          int dir = nextDir(0);
          LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
          nbr.putToken(std::make_shared<CandidateTreeDoneToken>(localToGlobalDir((dir+3)%6), token->ttl, token->traversed));
        }
        if (hasToken<ForestDoneToken>()) {
          takeToken<ForestDoneToken>();
          for (int dir : children) {
            LeaderElectionDeterministicParticle &nbr = nbrAtLabel(dir);
            nbr.putToken(std::make_shared<ForestDoneToken>(localToGlobalDir((dir+3)%6)));
          }
          state = State::Convexification;
        }
      }
    }
  }
  else if (state == State::Convexification) {
    // TODO
  }
}

int LeaderElectionDeterministicParticle::headMarkDir() const {
  return parent;
}

int LeaderElectionDeterministicParticle::headMarkColor() const {
  if (state == State::Initlialization) {
    if (segHeads.size() == 1) {
      if (segHeads[0]) {
        return 0xff9b00; // gold
      }
      else {
        return 0x7e7e7e; // gray
      }
    }
    else if (segHeads.size() > 1) {
      return 0xb900ff; // purple
    }
  }
  else if (state == State::ForestFormation) {
    return 0x008800; // dark green
  }
  else if (state == State::ForestFormationCandidate) {
     return 0x5a2d00; // brown
  }
  else if (state == State::Convexification) {
    return 0x0000ff; // blue
  }
  else if (state == State::Candidate) {
    return 0xff9b00; // gold
  }
  else if (state == State::Leader) {
    return 0x00ff00; // green
  }
  return -1;
}

LeaderElectionDeterministicParticle &
LeaderElectionDeterministicParticle::nbrAtLabel(int label) const {
  return AmoebotParticle::nbrAtLabel<LeaderElectionDeterministicParticle>(label);
}

QString LeaderElectionDeterministicParticle::inspectionText() const {
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
    default:
      return "no state";
    }
  }();
  text += "\n";
  text += "has leader election tokens: " +
          QString::number(countTokens<LeaderElectionToken>()) + "\n";
  text += "\n";

  return text;
}

bool LeaderElectionDeterministicParticle::isBoundaryParticle() {
  for (int dir = 0; dir < 6; dir++) {
    if (!hasNbrAtLabel(dir)) {
      return true;
    }
  }
  return false;
}

int LeaderElectionDeterministicParticle::numBoundaries() {
  int num = 0;
  for (int dir = 0; dir < 6; dir++) {
    int prevDir = (dir + 5) % 6;
    if (hasNbrAtLabel(prevDir) && !hasNbrAtLabel(dir)) {
      num += 1;
    }
  }
  return num;
}

void LeaderElectionDeterministicParticle::setLabels() {
  for (int dir = 0; dir < 6; dir++) {
    int prevDir = (dir + 5) % 6;
    if (hasNbrAtLabel(prevDir) && !hasNbrAtLabel(dir)) {
      int nextDir = (dir + 1) % 6;
      while (!hasNbrAtLabel(nextDir)) {
        nextDir = (nextDir + 1) % 6;
      }
      if (nextDir == (dir + 1) % 6) {
        labels.push_back(-1);
      }
      else if (nextDir == (dir + 2) % 6) {
        labels.push_back(0);
      }
      else if (nextDir == (dir + 3) % 6) {
        labels.push_back(1);
      }
      else if (nextDir == (dir + 4) % 6) {
        labels.push_back(2);
      }
      else if (nextDir == (dir + 5) % 6) {
        labels.push_back(3);
      }
      else {
        Q_ASSERT(false);
      }
    }
  }
}

int LeaderElectionDeterministicParticle::nextDir(int boundary) {
  int num = 0;
  for (int dir = 0; dir < 6; dir++) {
    int next = (dir + 5) % 6;
    if (hasNbrAtLabel(next) && !hasNbrAtLabel(dir)) {
      if (num == boundary) {
        return next;
      }
      num += 1;
    }
  }
}

int LeaderElectionDeterministicParticle::prevDir(int boundary) {
  int num = 0;
  for (int dir = 0; dir < 6; dir++) {
    int next = (dir + 5) % 6;
    if (hasNbrAtLabel(next) && !hasNbrAtLabel(dir)) {
      if (num == boundary) {
        int prev = (dir + 1) % 6;
        while (!hasNbrAtLabel(prev)) {
          prev = (prev + 1) % 6;
        }
        return prev;
      }
      num += 1;
    }
  }
}

void LeaderElectionDeterministicParticle::cleanup(int boundary) {
  // reset lexicographic comparison variables
  lexicoGraphicComparisons[boundary] = false;
  sentLabels[boundary] = false;
  reqLabels[boundary] = false;
  reqNbrLabels[boundary] = false;
  receivedLabels[boundary] = false;
  receivedNbrLabels[boundary] = false;
  internalLabels[boundary] = 0;
  nbrLabels[boundary] = 0;
  endOfSegments[boundary] = false;
  endOfNbrSegments[boundary] = false;
  firstLargerLabels[boundary] = 0;

  int nextNbr = nextDir(boundary);

  // pass cleanup token throughout segment
  if (successors[boundary] != -1) {
    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
    nbr.putToken(std::make_shared<LexCompCleanupToken>(localToGlobalDir((nextNbr+3)%6)));
  }

  while (hasToken<LexCompReturnNbrLabelToken>()){
    std::shared_ptr<LexCompReturnNbrLabelToken> token = peekAtToken<LexCompReturnNbrLabelToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompReturnNbrLabelToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompReturnNbrEndOfSegmentToken>()){
    std::shared_ptr<LexCompReturnNbrEndOfSegmentToken> token = peekAtToken<LexCompReturnNbrEndOfSegmentToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompReturnNbrEndOfSegmentToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompReturnLabelToken>()){
    std::shared_ptr<LexCompReturnLabelToken> token = peekAtToken<LexCompReturnLabelToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompReturnLabelToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompEndOfSegmentToken>()){
    std::shared_ptr<LexCompEndOfSegmentToken> token = peekAtToken<LexCompEndOfSegmentToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompEndOfSegmentToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompInterruptPrevToken>()){
    std::shared_ptr<LexCompInterruptPrevToken> token = peekAtToken<LexCompInterruptPrevToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompInterruptPrevToken>();
    }
    else {
      break;
    }
  }
}

void LeaderElectionDeterministicParticle::cleanupForNbr(int boundary) {
  // reset lexicographic comparison variables
  sentNbrLabels[boundary] = false;
  reqLabelsForNbr[boundary] = false;

  int nextNbr = nextDir(boundary);
  int prevNbr = prevDir(boundary);

  // pass cleanup token throughout segment
  if (successors[boundary] != -1) {
    LeaderElectionDeterministicParticle &nbr = nbrAtLabel(nextNbr);
    nbr.putToken(std::make_shared<LexCompCleanupForNbrToken>(localToGlobalDir((nextNbr+3)%6)));
  }

  while (hasToken<LexCompRequestNbrLabelToken>()){
    std::shared_ptr<LexCompRequestNbrLabelToken> token = peekAtToken<LexCompRequestNbrLabelToken>();
    if (globalToLocalDir(token->origin) == prevNbr && segHeads[boundary]) {
      takeToken<LexCompRequestNbrLabelToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompReturnLabelForNbrToken>()){
    std::shared_ptr<LexCompReturnLabelForNbrToken> token = peekAtToken<LexCompReturnLabelForNbrToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompReturnLabelForNbrToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompEndOfSegmentForNbrToken>()){
    std::shared_ptr<LexCompEndOfSegmentForNbrToken> token = peekAtToken<LexCompEndOfSegmentForNbrToken>();
    if (globalToLocalDir(token->origin) == nextNbr) {
      takeToken<LexCompEndOfSegmentForNbrToken>();
    }
    else {
      break;
    }
  }
  while (hasToken<LexCompInterruptNextToken>()){
    std::shared_ptr<LexCompInterruptNextToken> token = peekAtToken<LexCompInterruptNextToken>();
    if (globalToLocalDir(token->origin) == prevNbr) {
      takeToken<LexCompInterruptNextToken>();
    }
    else {
      break;
    }
  }
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN SYSTEM CODE----------------------------

#include <string>
#include <fstream>
#include <sstream>
#include <QTextStream>

using namespace std;

LeaderElectionDeterministicSystem::LeaderElectionDeterministicSystem(int numParticles, QString fileName) {
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

      insert(new LeaderElectionDeterministicParticle(
      Node(x, y), -1, randDir(), *this,
      LeaderElectionDeterministicParticle::State::Initlialization));
    }

    file.close();

    outputPath = "../AmoebotSim/data/output/" + fileName.toStdString() + ".txt";

    out << "Particle system initialized from file." << endl;
    
    return;
  }

  randomPermutationScheduler = true;
  randomReshuffleProb = 0.1;

  // Insert the seed at (0,0).
  insert(new LeaderElectionDeterministicParticle(
      Node(0, 0), -1, randDir(), *this,
      LeaderElectionDeterministicParticle::State::Initlialization));
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
          insert(new LeaderElectionDeterministicParticle(
              nbr, -1, randDir(), *this,
              LeaderElectionDeterministicParticle::State::Initlialization));
          ++added;
          if (added == numParticles) {
            break;
          }
        }
      }
    }
  }
}

bool LeaderElectionDeterministicSystem::hasTerminated() const {
#ifdef QT_DEBUG
  if (!isConnected(particles)) {
    return true;
  }
#endif

  for (auto p : particles) {
    auto hp = dynamic_cast<LeaderElectionDeterministicParticle *>(p);
    if (hp->state == LeaderElectionDeterministicParticle::State::Leader) {
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