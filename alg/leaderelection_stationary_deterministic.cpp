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

#include <qDebug>

using namespace std;

//----------------------------BEGIN PARTICLE CODE----------------------------

LeaderElectionStationaryDeterministicParticle::LeaderElectionStationaryDeterministicParticle(
    const Node head, const int globalTailDir, const int orientation,
    AmoebotSystem &system, State state)
    : AmoebotParticle(head, globalTailDir, orientation, system), state(state) {
  borderColorLabels.fill(-1);
  borderPointColorLabels.fill(-1);
  borderPointBetweenEdgeColorLabels.fill(-1);
  borderHalfPointBetweenEdgeColorLabels.fill(-1);
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
      state = State::Demoted;
    }
    else {
      // Initialize 6 nodes
      qDebug() << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Initializing nodes... @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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
      state = State::Demoted;
      return;
    }
    // Wait for all neighbors to reach stretch expansion state.
    for (int dir = 0; dir < 6; dir++) {
      if (hasNbrAtLabel(dir)) {
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
        if (nbr.state == State::IdentificationLabeling) {
          return;
        }
        // If tree formation phase is starting, join tree
        else if (nbr.state == State::Candidate || nbr.state == State::TreeFormation) {
          for (LeaderElectionNode* node : nodes) {
            if (node->predecessor == nullptr) {
              // If this particle has a node that is the head of a stretch
              // Then become a candidate
              state = State::Candidate;
              tree = true;
              headCount = node->count;
              return;
            }
          }
          state = State::TreeFormation;
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
          node->lexCompTryMerge = clone->lexCompTryMerge;

          node->terminationDetectionInitiated = clone->terminationDetectionInitiated;

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
          node->lexCompTryMerge = clone->lexCompTryMerge;

          node->terminationDetectionInitiated = clone->terminationDetectionInitiated;

          clone->cloneChange = false;
        }
        node->cloneChange = true;
      }
    }
    // Activate each node. Nodes run stretch expansion.
    for (int i = 0; i < nodes.size(); i++){
      nodes.at(i)->activate();
    }
    for (int i = 0; i < nodes.size(); i++){
      LeaderElectionNode* node = nodes.at(i);
      if (node->mergePending) {
        node->paintNode(0xb900ff); // Purple
      }
      else if (node->terminationDetectionInitiated && node->predecessor == nullptr) {
        node->paintNode(0xff0000); // Red
      }
      else if (node->lexicographicComparisonRight) {
        node->paintNode(0x00ff00); // Green
      }
      else if (node->predecessor == nullptr) {
        node->paintNode(0xff9b00); // Gold
      }
      else if (node->hasNodeToken<TerminationDetectionToken>(node->nextNode()->prevNodeDir)) {
        node->paintNode(0x00aeff); // Light blue
      }
      else if (node->hasNodeToken<TerminationDetectionReturnToken>(node->prevNode()->nextNodeDir)) {
        node->paintNode(0x00aeff); // Light blue
      }
      else {
        node->paintNode(0x000000); // Black
      }
    }
  }
  else if (state == State::Demoted) {
    for (int dir = 0; dir < 6; dir++) {
      if (hasNbrAtLabel(dir)) {
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
        // If tree formation phase is starting, change state
        if (nbr.state == State::Candidate || (nbr.state == State::TreeFormation && nbr.tree)) {
          state = State::TreeFormation;
          tree = true;
          parent = dir;
          nbr.putToken(std::make_shared<ParentToken>(localToGlobalDir(parent)));
          return;
        }
      }
    }
  }
  else if (state == State::TreeFormation) {
    // qDebug() << "TreeFormation particle running...";
    // process and pass cleanup tokens
    if (hasToken<CleanUpToken>()) {
      qDebug() << "Processing cleanup token...";
      takeToken<CleanUpToken>();
      treeDone = false;
      nbrhdEncodingSentRight = false;
      nbrhdEncodingSentLeft = false;
      treeExhaustedRight = false;
      treeExhaustedLeft = false;
      childrenExhaustedRight = {};
      childrenExhaustedLeft = {};
      for (int childDir : children) {
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
        child.putToken(std::make_shared<CleanUpToken>(localToGlobalDir(childDir)));
      }
      return;
    }
    // pass comparison result tokens
    while (hasToken<ComparisonResultToken>()) {
      std::shared_ptr<ComparisonResultToken> token = takeToken<ComparisonResultToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<ComparisonResultToken>(localToGlobalDir(nextDir), token->ttl, token->traversed, token->result));
    }
    // receive parent tokens -> add to children
    while (hasToken<ParentToken>()) {
      qDebug() << "Processing parent token...";
      std::shared_ptr<ParentToken> token = takeToken<ParentToken>();
      int globalParentDir = token->origin;
      int localParentDir = globalToLocalDir(globalParentDir);
      int localChildDir = (localParentDir + 3) % 6;
      children.insert(localChildDir);
    }
    // If not in tree, attempt to join a tree
    if (!tree) {
      qDebug() << "Not in tree...";
      // for particles on the border
      // receive childTokens from stretch predecessor to join the tree
      // with the head of the stretch as root
      if (hasToken<ChildToken>()) {
        qDebug() << "Processing child token...";
        std::shared_ptr<ChildToken> token = takeToken<ChildToken>();
        int globalParentDir = token->origin;
        int localParentDir = globalToLocalDir(globalParentDir);
        parent = (localParentDir + 3) % 6;
        LeaderElectionStationaryDeterministicParticle &p = nbrAtLabel(parent);
        p.putToken(std::make_shared<ParentToken>(localToGlobalDir(parent)));
        tree = true;

        // Forward childToken to remainder of stretch if applicable
        for (LeaderElectionNode* node : nodes) {
          if (node->prevNodeDir == parent) {
            while (node->nextNodeDir < 0) {
              node = node->nextNode();
            }
            LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(node->nextNodeDir);
            if (!nbr.tree) {
              qDebug() << "Forwarding child token...";
              nbr.putToken(std::make_shared<ChildToken>(localToGlobalDir(node->nextNodeDir)));
            }
            break;
          }
        }
      }
      // If not on the border, then join any adjacent particle's tree
      else if (nodes.size() == 0) {
        qDebug() << "Attempting to join a tree...";
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
            if (nbr.tree) {
              parent = dir;
              nbr.putToken(std::make_shared<ParentToken>(localToGlobalDir(parent)));
              tree = true;
              break;
            }
          }
        }
      }
      // particles on the border that have not received a child token...
      /*
      else {
        int dir;
        for (dir = 0; dir < 6; dir++) {
          if (!hasNbrAtLabel(dir)) {
            break;
          }
        }
        while (!hasNbrAtLabel(dir)) {
          dir = (dir + 1) % 6;
        }
        parent = dir;
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
        nbr.putToken(std::make_shared<ParentToken>(localToGlobalDir(parent)));
        tree = true;
      }
      */
    }
    // if no non-tree neighbours and all children have treeDone -> set treeDone
    if (!treeDone && tree) {
      qDebug() << "In tree but not treeDone...";
      bool done = true;
      for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
          if (!nbr.tree) {
            done = false;
          }
        }
      }
      for (int dir : children) {
        if (dir == parent) {
          continue;
        }
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(dir);
        if (!child.treeDone || child.hasToken<CleanUpToken>()) {
          done = false;
        }
      }
      if (done) {
        treeDone = true;
      }
    }
    // receive TreeComparisonStartTokens and advance to next phase
    if (hasToken<TreeComparisonStartToken>()) {
      qDebug() << "Processing tree comparison start token...";
      takeToken<TreeComparisonStartToken>();
      state = State::TreeComparison;
      for(int childDir : children) {
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
        child.putToken(std::make_shared<TreeComparisonStartToken>(localToGlobalDir(childDir)));
      }
    }
    else if (parent >= 0 && treeDone) {
      if ((nbrAtLabel(parent).treeFormationDone || nbrAtLabel(parent).state == State::TreeComparison)) {
        qDebug() << "Changing state to TreeComparison...";
        state = State::TreeComparison;
        for(int childDir : children) {
          LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
          child.putToken(std::make_shared<TreeComparisonStartToken>(localToGlobalDir(childDir)));
        }
        return;
      }
      if (children.find(parent) != children.end()) {
        for (int dir = 0; dir < 6; dir++) {
          if (hasNbrAtLabel(dir)) {
            LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
            if (nbr.state == State::TreeComparison || nbr.treeFormationDone) {
              qDebug() << "Changing state to TreeComparison...";
              state = State::TreeComparison;
              for(int childDir : children) {
                LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
                child.putToken(std::make_shared<TreeComparisonStartToken>(localToGlobalDir(childDir)));
              }
              return;
            }
          }
        }
      }
    }
    // receive and pass TreeFormationFinishedTokens
    while (hasToken<TreeFormationFinishedToken>()) {
      qDebug() << "Processing tree formation finished token...";
      std::shared_ptr<TreeFormationFinishedToken> token = takeToken<TreeFormationFinishedToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<TreeFormationFinishedToken>(localToGlobalDir(nextDir), token->ttl, token->traversed));
    }
  }
  else if (state == State::Candidate) {
    qDebug() << "Candidate particle running... (" << QString::number(head.x) << ", " << QString::number(head.y) << ")";
    // Set the number of candidates when a candidate is activated for the first time
    if (numCandidates == 0) {
      numCandidates = 6 / headCount;
    }
    // receive parent tokens -> add to children
    while (hasToken<ParentToken>()) {
      qDebug() << "Processing parent token...";
      std::shared_ptr<ParentToken> token = takeToken<ParentToken>();
      int globalParentDir = token->origin;
      int localParentDir = globalToLocalDir(globalParentDir);
      int localChildDir = (localParentDir + 3) % 6;
      children.insert(localChildDir);
    }
    // send childTokens to particle(s) containing next node of the stretch
    if (!childTokensSent) {
      qDebug() << "Sending child tokens...";
      for (int i = 0; i < nodes.size(); i++) {
        LeaderElectionNode* node = nodes[i];
        if (node->predecessor == nullptr) {
          while (node->nextNodeDir < 0) {
            i = (i + 1) % nodes.size();
            node = nodes[i];
          }
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(node->nextNodeDir);
          nextDirCandidate = node->nextNodeDir;
          if (!nbr.tree) {
            nbr.putToken(std::make_shared<ChildToken>(localToGlobalDir(node->nextNodeDir)));
          }
          break;
        }
      }
      childTokensSent = true;
    }
    if (treeComparisonReady) {
      // Send comparison result token to all other candidates
      if (comparisonDone && !comparisonSent) {
        qDebug() << "Sending comparison result...";
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
        nbr.putToken(std::make_shared<ComparisonResultToken>(localToGlobalDir(nextDirCandidate), numCandidates, 1, comparisonResult));
        comparisonSent = true;
        std::vector<int> newVector(numCandidates, comparisonResult);
        comparisonResults = newVector;
        comparisonsReceived = 1;
      }
      // receive comparison results from other candidates
      if (comparisonDone && comparisonsReceived < numCandidates) {
        while (hasToken<ComparisonResultToken>()) {
          qDebug() << "Processing comparison result token...";
          std::shared_ptr<ComparisonResultToken> token = takeToken<ComparisonResultToken>();
          int index = 1 + (token->ttl - (token->traversed + 1));
          comparisonResults[index] = token->result;
          comparisonsReceived += 1;
          // pass the token on if necessary
          if (token->traversed + 1 < token->ttl) {
            LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
            nbr.putToken(std::make_shared<ComparisonResultToken>(localToGlobalDir(nextDirCandidate), token->ttl, token->traversed+1, token->result));
          }
        }
      }
      // Eliminate candidates based on received comparison results
      if (comparisonDone && comparisonsReceived == numCandidates) {
        qDebug() << "Processing comparison results...";
        for (int res : comparisonResults) {
          qDebug() << QString::number(res);
        }
        set<std::vector<int>> seqs = getMaxNonDescSubSeq(comparisonResults);
        for (std::vector<int> seq : seqs) {
          qDebug() << "Processing maximal non-descending subsequence...";
          for (int s : seq) {
            qDebug() << QString::number(s);
          }
          // If all candidates are equal, there is unbreakable symmetry
          if (seq.size() == numCandidates + 1) {
            state = State::Finished;
            return;
          }
          int candidate = seq[0]; // candidate to be eliminated
          // If this candidate is eliminated, revoke candidacy, join tree
          // of candidate to the left
          if (candidate == 0) {
            state = State::TreeFormation;
            tree = true;
            parent = (nextDirCandidate + 1) % 6;
            while (!hasNbrAtLabel(parent)) {
              parent = (parent + 1) % 6;
            }
            LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(parent);
            nbr.putToken(std::make_shared<ParentToken>(localToGlobalDir(parent)));
            for (int childDir : children) {
              LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
              child.putToken(std::make_shared<CleanUpToken>(localToGlobalDir(childDir)));
            }
            treeDone = false;
            treeFormationDone = false;
            treeExhaustedRight = false;
            treeExhaustedLeft = false;
            encodingReceivedRight = false;
            encodingReceivedLeft = false;
            encodingRequestedRight = false;
            encodingRequestedLeft = false;
            nbrhdEncodingSentRight = false;
            nbrhdEncodingSentLeft = false;
            childrenExhaustedRight = {};
            childrenExhaustedLeft = {};
            return;
          }
          else {
            numCandidates -= 1;
            // If this candidate is now the last remaining candidate, become the leader
            if (numCandidates == 1) {
              state = State::Leader;
              return;
            }
          }
        }
        // clean up comparison variables in candidate and subtree
        treeDone = false;
        treeFormationDone = false;
        treeComparisonReady = false;
        comparisonDone = false;
        comparisonsReceived = 0;
        comparisonSent = false;
        childrenExhaustedRight = {};
        childrenExhaustedLeft = {};
        nbrhdEncodingSentRight = false;
        nbrhdEncodingSentLeft = false;
        nbrEncodingRequestReceived = false;
        encodingRequestedRight = false;
        encodingRequestedLeft = false;
        nbrEncodingRequested = false;
        encodingReceivedRight = false;
        encodingReceivedLeft = false;
        nbrEncodingReceived = false;
        treeExhaustedRight = false;
        treeExhaustedLeft = false;
        nbrTreeExhausted = false;
        for (int childDir : children) {
          LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
          child.putToken(std::make_shared<CleanUpToken>(localToGlobalDir(childDir)));
        }
        return;
      }
      // receive encoding request tokens from other candidates
      while (hasToken<RequestCandidateEncodingToken>()) {
        qDebug() << "Processing candidate encoding request token...";
        std::shared_ptr<RequestCandidateEncodingToken> token = takeToken<RequestCandidateEncodingToken>();
        // If token is intended for other candidate, pass it on
        if (token->traversed + 1 != token->ttl) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
          nbr.putToken(std::make_shared<RequestCandidateEncodingToken>(localToGlobalDir(nextDirCandidate), token->ttl, token->traversed+1));
        }
        else {
          nbrEncodingRequestReceived = true;
        }
      }

      // receive tree exhausted tokens from other candidates
      while (hasToken<CandidateTreeExhaustedToken>()) {
        qDebug() << "Processing candidate tree exhausted token...";
        std::shared_ptr<CandidateTreeExhaustedToken> token = takeToken<CandidateTreeExhaustedToken>();
        // if token is intended for other candidate, pass it on
        if (token->traversed + 1 != token->ttl) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
          nbr.putToken(std::make_shared<CandidateTreeExhaustedToken>(localToGlobalDir(nextDirCandidate), token->ttl, token->traversed+1));
        }
        // If token is intended for this candidate, store it
        else {
          nbrTreeExhausted = true;
          nbrEncodingReceived = true;
          nbrEncodingRequested = false;
        }
      }
      // Receive neighbourhood encodings from other candidates
      while (hasToken<CandidateEncodingToken>()) {
        qDebug() << "Processing candidate encoding token...";
        std::shared_ptr<CandidateEncodingToken> token = takeToken<CandidateEncodingToken>();
        // if token is intended for other candidate, pass it on
        if (token->traversed + 1 != token->ttl) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
          nbr.putToken(std::make_shared<CandidateEncodingToken>(localToGlobalDir(nextDirCandidate), token->ttl, token->traversed+1, token->encoding));
        }
        // If token is intended for this candidate, store it
        else {
          currentEncodingNbr = token->encoding;
          nbrEncodingReceived = true;
          nbrEncodingRequested = false;
        }
      }

      // request encoding from right stretch
      if (!nbrEncodingRequested && !nbrEncodingReceived && !comparisonDone) {
        qDebug() << "Requesting encoding from right stretch...";
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
        nbr.putToken(std::make_shared<RequestCandidateEncodingToken>(localToGlobalDir(nextDirCandidate), 2, 1));
        nbrEncodingRequested = true;
      }
      // request encodings from tree for comparison with right stretch
      if (!encodingRequestedRight && !encodingReceivedRight && !comparisonDone) {
        qDebug() << "Requesting right encoding from tree...";
        // first step: use own neighborhood encoding
        if (!nbrhdEncodingSentRight) {
          currentEncodingRight = getNeighborhoodEncoding();
          encodingReceivedRight = true;
          nbrhdEncodingSentRight = true;
        }
        // Otherwise request encoding from tree
        else if (children.size() > 0) {
          // Loop through children in direction of increasing labels
          // Starting from empty node(s)
          int childDir = nextDirCandidate;
          while (children.find(childDir) == children.end() || childrenExhaustedRight.find(childDir) != childrenExhaustedRight.end()) {
            childDir = (childDir + 1) % 6;
            if (childDir == nextDirCandidate) {
              break;
            }
          }
          // tree exhausted
          if (childDir == nextDirCandidate && childrenExhaustedRight.find(childDir) != childrenExhaustedRight.end()) {
            treeExhaustedRight = true;
            encodingRequestedRight = false;
            encodingReceivedRight = true;
          }
          else {
            LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
            child.putToken(std::make_shared<RequestEncodingRightToken>(localToGlobalDir(childDir)));
            encodingRequestedRight = true;
          }
        }
        else {
          // tree exhausted
          treeExhaustedRight = true;
          encodingRequestedRight = false;
          encodingReceivedRight = true;
        }
      }
      // receive encodings from tree for comparison with right stretch
      if (encodingRequestedRight && !encodingReceivedRight) {
        if (hasToken<EncodingRightToken>()) {
          qDebug() << "Processing right encoding token...";
          std::shared_ptr<EncodingRightToken> token = takeToken<EncodingRightToken>();
          currentEncodingRight = token->encoding;
          encodingRequestedRight = false;
          encodingReceivedRight = true;
        }
        // receive subtree exhausted tokens
        else if (hasToken<SubTreeExhaustedRightToken>()) {
          qDebug() << "Processing right subtree exhausted token...";
          std::shared_ptr<SubTreeExhaustedRightToken> token = takeToken<SubTreeExhaustedRightToken>();
          int dir = (globalToLocalDir(token->origin) + 3) % 6;
          childrenExhaustedRight.insert(dir);
          encodingRequestedRight = false;
        }
      }
      // compare both encodings
      if (encodingReceivedRight && nbrEncodingReceived) {
        qDebug() << "Comparing encodings...";
        // own tree exhausted -> smaller
        if (treeExhaustedRight && !nbrTreeExhausted) {
          comparisonResult = -1;
        }
        // right stretch tree exhausted -> larger
        else if (!treeExhaustedRight && nbrTreeExhausted) {
          comparisonResult = 1;
        }
        // both trees exhausted -> equal
        else if (treeExhaustedRight && nbrTreeExhausted) {
          comparisonResult = 0;
        }
        // neither exhausted -> compare encodings
        else {
          if (currentEncodingRight > currentEncodingNbr) {
            comparisonResult = 1;
          }
          else if (currentEncodingRight < currentEncodingNbr) {
            comparisonResult = -1;
          }
          else {
            comparisonResult = 0;
          }
        }
        // Encodings equal -> request next
        if (comparisonResult == 0 && !treeExhaustedRight) {
          encodingReceivedRight = false;
          nbrEncodingReceived = false;
        }
        // equal trees -> comparison done
        else if (comparisonResult == 0 && treeExhaustedRight) {
          comparisonDone = true;
        }
        // Different encodings -> comparison done
        else {
          comparisonDone = true;
        }
        encodingReceivedRight = false;
        nbrEncodingReceived = false;
      }

      // request encoding from tree for left stretch
      if (nbrEncodingRequestReceived && !encodingRequestedLeft && !encodingReceivedLeft) {
        qDebug() << "Requesting left encoding from tree...";
        // first step: use own neighborhood encoding
        if (!nbrhdEncodingSentLeft) {
          currentEncodingLeft = getNeighborhoodEncoding();
          encodingReceivedLeft = true;
          nbrhdEncodingSentLeft = true;
        }
        // Otherwise request encoding from tree
        else if (children.size() > 0) {
          // Loop through children in direction of increasing labels
          // Starting from empty node(s)
          int childDir = nextDirCandidate;
          while (children.find(childDir) == children.end() || childrenExhaustedLeft.find(childDir) != childrenExhaustedLeft.end()) {
            childDir = (childDir + 1) % 6;
            if (childDir == nextDirCandidate) {
              break;
            }
          }
          // tree exhausted
          if (childDir == nextDirCandidate && childrenExhaustedLeft.find(childDir) != childrenExhaustedLeft.end()) {
            treeExhaustedLeft = true;
            encodingRequestedLeft = false;
            encodingReceivedLeft = true;
          }
          else {
            LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
            child.putToken(std::make_shared<RequestEncodingLeftToken>(localToGlobalDir(childDir)));
            encodingRequestedLeft = true;
          }
        }
        else {
          // tree exhausted
          treeExhaustedLeft = true;
          encodingRequestedLeft = false;
          encodingReceivedLeft = true;
        }
      }
      // receive requested encoding from tree for left stretch
      if (nbrEncodingRequestReceived && encodingRequestedLeft && !encodingReceivedLeft) {
        if (hasToken<EncodingLeftToken>()) {
          qDebug() << "Processing left encoding token...";
          std::shared_ptr<EncodingLeftToken> token = takeToken<EncodingLeftToken>();
          currentEncodingLeft = token->encoding;
          encodingRequestedLeft = false;
          encodingReceivedLeft = true;
        }
        // receive subtree exhausted tokens
        else if (hasToken<SubTreeExhaustedLeftToken>()) {
          qDebug() << "Processing left subtree exhausted token...";
          std::shared_ptr<SubTreeExhaustedLeftToken> token = takeToken<SubTreeExhaustedLeftToken>();
          int dir = (globalToLocalDir(token->origin) + 3) % 6;
          childrenExhaustedLeft.insert(dir);
          encodingRequestedLeft = false;
        }
      }
      // send encodings to left stretch
      if (nbrEncodingRequestReceived && encodingReceivedLeft) {
        qDebug() << "Sending encoding to left stretch...";
        if (!treeExhaustedLeft) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
          nbr.putToken(std::make_shared<CandidateEncodingToken>(localToGlobalDir(nextDirCandidate), numCandidates, 1, currentEncodingLeft));
          nbrEncodingRequestReceived = false;
          encodingReceivedLeft = false;
        }
        else {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
          nbr.putToken(std::make_shared<CandidateTreeExhaustedToken>(localToGlobalDir(nextDirCandidate), numCandidates, 1));
          nbrEncodingRequestReceived = false;
          encodingReceivedLeft = false;
        }
      }
    }
    // if no non-tree neighbours and all children have treeDone -> set treeDone
    if (!treeDone) {
      qDebug() << "Evaluating treeDone...";
      bool done = true;
      for (int dir = 0; dir < 6; dir++) {
        if (hasNbrAtLabel(dir)) {
          LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
          if (!nbr.tree) {
            done = false;
          }
        }
      }
      for (int dir : children) {
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(dir);
        if (!child.treeDone || child.hasToken<CleanUpToken>()) {
          done = false;
        }
      }
      if (done) {
        treeDone = true;
      }
    }
    // when treeDone, send tokens throughout tree and to other candidates to move to comparison phase
    if (treeDone && !treeFormationDone) {
      qDebug() << "Sending TreeComparisonStartTokens...";
      // send tokens to other candidates to communicate that tree formation is finished
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
      nbr.putToken(std::make_shared<TreeFormationFinishedToken>(localToGlobalDir(nextDirCandidate), numCandidates, 1));
      treeFormationFinishedTokensReceived = 1;
      // Send tokens to children to indicate that tree formation is done and comparison will start soon
      for (int childDir : children) {
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
        if (child.state != State::TreeComparison) {
          child.putToken(std::make_shared<TreeComparisonStartToken>(localToGlobalDir(childDir)));
        }
      }
      treeFormationDone = true;
      qDebug() << "Tree formation done";
    }
    // receive TreeFormationFinishedTokens and pass them on if applicable
    while (hasToken<TreeFormationFinishedToken>() && treeFormationDone && !treeComparisonReady) {
      qDebug() << "Processing TreeFormationFinishedToken...";
      std::shared_ptr<TreeFormationFinishedToken> token = takeToken<TreeFormationFinishedToken>();
      treeFormationFinishedTokensReceived += 1;
      // pass token if necessary
      if (token->traversed + 1 < token->ttl) {
        LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDirCandidate);
        nbr.putToken(std::make_shared<TreeFormationFinishedToken>(localToGlobalDir(nextDirCandidate), token->ttl, token->traversed+1));
      }
    }
    if (treeFormationFinishedTokensReceived >= numCandidates) {
      treeComparisonReady = true;
      treeFormationFinishedTokensReceived = 0;
      qDebug() << "Ready for tree comparison";
    }
  }
  else if (state == State::TreeComparison) {
    qDebug() << "TreeComparison particle running...";
    if (!treeDone) {
      state = State::TreeFormation;
    }
    // process and pass cleanup tokens
    if (hasToken<CleanUpToken>()) {
      takeToken<CleanUpToken>();
      treeDone = false;
      state = State::TreeFormation;
      nbrhdEncodingSentRight = false;
      nbrhdEncodingSentLeft = false;
      treeExhaustedRight = false;
      treeExhaustedLeft = false;
      childrenExhaustedRight = {};
      childrenExhaustedLeft = {};
      for (int childDir : children) {
        LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
        child.putToken(std::make_shared<CleanUpToken>(localToGlobalDir(childDir)));
      }
      return;
    }
    // pass tree formation finished tokens
    while (hasToken<TreeFormationFinishedToken>()) {
      std::shared_ptr<TreeFormationFinishedToken> token = takeToken<TreeFormationFinishedToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<TreeFormationFinishedToken>(localToGlobalDir(nextDir), token->ttl, token->traversed));
    }
    // pass comparison result tokens
    while (hasToken<ComparisonResultToken>()) {
      std::shared_ptr<ComparisonResultToken> token = takeToken<ComparisonResultToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<ComparisonResultToken>(localToGlobalDir(nextDir), token->ttl, token->traversed, token->result));
    }
    // pass candidate encoding requests
    while (hasToken<RequestCandidateEncodingToken>()) {
      std::shared_ptr<RequestCandidateEncodingToken> token = takeToken<RequestCandidateEncodingToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<RequestCandidateEncodingToken>(localToGlobalDir(nextDir), token->ttl, token->traversed));
    }
    // pass candidate tree exhausted tokens
    while (hasToken<CandidateTreeExhaustedToken>()) {
      std::shared_ptr<CandidateTreeExhaustedToken> token = takeToken<CandidateTreeExhaustedToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<CandidateTreeExhaustedToken>(localToGlobalDir(nextDir), token->ttl, token->traversed));
    }
    // pass candidate encoding tokens
    while (hasToken<CandidateEncodingToken>()) {
      std::shared_ptr<CandidateEncodingToken> token = takeToken<CandidateEncodingToken>();
      int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(nextDir);
      nbr.putToken(std::make_shared<CandidateEncodingToken>(localToGlobalDir(nextDir), token->ttl, token->traversed, token->encoding));
    }
    // process and/or pass right encoding requests
    if (hasToken<RequestEncodingRightToken>()) {
      std::shared_ptr<RequestEncodingRightToken> token = takeToken<RequestEncodingRightToken>();
      // first step: use own neighbourhood encoding
      if (!nbrhdEncodingSentRight) {
        currentEncodingRight = getNeighborhoodEncoding();
        encodingReceivedRight = true;
        nbrhdEncodingSentRight = true;
      }
      // Otherwise request encoding from tree
      else if (children.size() > 0) {
        // Loop through children in direction of increasing labels
        // Starting from empty node(s)
        int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
        int childDir = nextDir;
        while (children.find(childDir) == children.end() || childrenExhaustedRight.find(childDir) != childrenExhaustedRight.end()) {
          childDir = (childDir + 1) % 6;
          if (childDir == nextDir) {
            break;
          }
        }
        // tree exhausted
        if (childDir == nextDir && childrenExhaustedRight.find(childDir) != childrenExhaustedRight.end()) {
          treeExhaustedRight = true;
          encodingReceivedRight = true;
        }
        else {
          LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
          child.putToken(std::make_shared<RequestEncodingRightToken>(localToGlobalDir(childDir)));
          encodingRequestedRight = true;
        }
      }
      else {
        // tree exhausted
        treeExhaustedRight = true;
        encodingReceivedRight = true;
      }
    }
    // receive encodings from tree for comparison with right stretch
    if (encodingRequestedRight && !encodingReceivedRight) {
      if (hasToken<EncodingRightToken>()) {
        std::shared_ptr<EncodingRightToken> token = takeToken<EncodingRightToken>();
        currentEncodingRight = token->encoding;
        encodingRequestedRight = false;
        encodingReceivedRight = true;
      }
      // receive subtree exhausted tokens
      else if (hasToken<SubTreeExhaustedRightToken>()) {
        std::shared_ptr<SubTreeExhaustedRightToken> token = takeToken<SubTreeExhaustedRightToken>();
        int dir = (globalToLocalDir(token->origin) + 3) % 6;
        childrenExhaustedRight.insert(dir);
        encodingRequestedRight = false;
      }
    }
    // send received encodings towards root
    if (encodingReceivedRight) {
      // if tree exhausted, send subtree exhausted token
      if (treeExhaustedRight) {
        LeaderElectionStationaryDeterministicParticle &p = nbrAtLabel(parent);
        p.putToken(std::make_shared<SubTreeExhaustedRightToken>(localToGlobalDir(parent)));
      }
      else {
        LeaderElectionStationaryDeterministicParticle &p = nbrAtLabel(parent);
        p.putToken(std::make_shared<EncodingRightToken>(localToGlobalDir(parent), currentEncodingRight));
      }
      encodingReceivedRight = false;
    }
    // process and/or pass left encoding requests
    if (hasToken<RequestEncodingLeftToken>()) {
      std::shared_ptr<RequestEncodingLeftToken> token = takeToken<RequestEncodingLeftToken>();
      // first step: use own neighbourhood encoding
      if (!nbrhdEncodingSentLeft) {
        currentEncodingLeft = getNeighborhoodEncoding();
        encodingReceivedLeft = true;
        nbrhdEncodingSentLeft = true;
      }
      // Otherwise request encoding from tree
      else if (children.size() > 0) {
        // Loop through children in direction of increasing labels
        // Starting from empty node(s)
        int nextDir = getNextDir((globalToLocalDir(token->origin) + 3) % 6);
        int childDir = nextDir;
        while (children.find(childDir) == children.end() || childrenExhaustedLeft.find(childDir) != childrenExhaustedLeft.end()) {
          childDir = (childDir + 1) % 6;
          if (childDir == nextDir) {
            break;
          }
        }
        // tree exhausted
        if (childDir == nextDir && childrenExhaustedLeft.find(childDir) != childrenExhaustedLeft.end()) {
          treeExhaustedLeft = true;
          encodingReceivedLeft = true;
        }
        else {
          LeaderElectionStationaryDeterministicParticle &child = nbrAtLabel(childDir);
          child.putToken(std::make_shared<RequestEncodingLeftToken>(localToGlobalDir(childDir)));
          encodingRequestedLeft = true;
        }
      }
      else {
        // tree exhausted
        treeExhaustedLeft = true;
        encodingReceivedLeft = true;
      }
    }
    // receive encodings from tree for comparison with left stretch
    if (encodingRequestedLeft && !encodingReceivedLeft) {
      if (hasToken<EncodingLeftToken>()) {
        std::shared_ptr<EncodingLeftToken> token = takeToken<EncodingLeftToken>();
        currentEncodingLeft = token->encoding;
        encodingRequestedLeft = false;
        encodingReceivedLeft = true;
      }
      // receive subtree exhausted tokens
      else if (hasToken<SubTreeExhaustedLeftToken>()) {
        std::shared_ptr<SubTreeExhaustedLeftToken> token = takeToken<SubTreeExhaustedLeftToken>();
        int dir = (globalToLocalDir(token->origin) + 3) % 6;
        childrenExhaustedLeft.insert(dir);
        encodingRequestedLeft = false;
      }
    }
    // send received encodings towards root
    if (encodingReceivedLeft) {
      // if tree exhausted, send subtree exhausted token
      if (treeExhaustedLeft) {
        LeaderElectionStationaryDeterministicParticle &p = nbrAtLabel(parent);
        p.putToken(std::make_shared<SubTreeExhaustedLeftToken>(localToGlobalDir(parent)));
      }
      else {
        LeaderElectionStationaryDeterministicParticle &p = nbrAtLabel(parent);
        p.putToken(std::make_shared<EncodingLeftToken>(localToGlobalDir(parent), currentEncodingLeft));
      }
      encodingReceivedLeft = false;
    }
  }
  return;
}

int LeaderElectionStationaryDeterministicParticle::headMarkDir() const {
  return parent;
}

int LeaderElectionStationaryDeterministicParticle::headMarkColor() const {
  if (state == State::IdentificationLabeling) {
    return 0x7e7e7e; // gray
  }
  else if (state == State::Demoted) {
    return 0xd2d2d2; // light gray
  }
  else if (state == State::Finished) {
    return 0xff0000; // red
  }
  else if (state == State::Leader) {
    return 0x00ff00; // green
  }
  else if (state == State::Candidate) {
    if (treeComparisonReady) {
      return 0x5a2d00; // brown
    }
    return 0xff9b00; // gold
  }
  else if (state == State::TreeFormation) {
    return 0x00b000; // green
  }
  else if (state == State::TreeComparison) {
    return 0x006100; // dark green
  }
  else {
    return -1;
  }
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

  if (state == State::StretchExpansion) {
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
  }
  else {
    text += "numCandidates: " + QString::number(numCandidates) + "\n";
    text += "nextDirCandidate: " + QString::number(nextDirCandidate) + "\n";
    text += "parent: " + QString::number(parent) + "\n";
    if (tree) {
      text += "tree: true\n";
    }
    else {
      text += "tree: false\n";
    }
    if (treeDone) {
      text += "treeDone: true\n";
    }
    else {
      text += "treeDone: false\n";
    }
    if (treeFormationDone) {
      text += "treeFormationDone: true\n";
    }
    else {
      text += "treeFormationDone: false\n";
    }
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

std::array<int, 12> LeaderElectionStationaryDeterministicParticle::borderHalfPointBetweenEdgeColors() const {
  return borderHalfPointBetweenEdgeColorLabels;
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

string LeaderElectionStationaryDeterministicParticle::getNeighborhoodEncoding() {
  string result = "";
  for (int dir = 0; dir < 6; ++dir) {
    if (hasNbrAtLabel(dir)) {
      LeaderElectionStationaryDeterministicParticle &nbr = nbrAtLabel(dir);
      if (nbr.state == State::Candidate) {
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

int LeaderElectionStationaryDeterministicParticle::getNextDir(int prevDir) {
  int nextDir = (prevDir + 5) % 6;
  while (!hasNbrAtLabel(nextDir)) {
    nextDir = (nextDir + 5) % 6;
  }
  return nextDir;
}

set<std::vector<int>> LeaderElectionStationaryDeterministicParticle::getMaxNonDescSubSeq(std::vector<int> input) {
  set<std::vector<int>> results = {{0}};
  for (int i = 0; i < input.size(); i++) {
    int comp = input[i];
    // If this comparison is ">", then a maximal non-descending subsequence
    // cannot start at this candidate
    if (comp == 1) {
      continue;
    }
    // If comparison is "=" or "<", then compute the subsequence starting at this candidate
    else {
      int index = (i + 1) % input.size();
      std::vector<int> sequence = {i, index};
      while (input[index] < 1 && index != i) {
        index = (index + 1) % input.size();
        sequence.push_back(index);
      }
      // If this sequence is the same as the largest so far, add it to the set
      if (sequence.size() == results.begin()->size()) {
        results.insert(sequence);
      }
      // If this sequence is larger, replace the set
      else if (sequence.size() > results.begin()->size()) {
        results = {sequence};
      }
    }
  }
  return results;
}

//----------------------------END PARTICLE CODE----------------------------

//----------------------------BEGIN AGENT CODE----------------------------

LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::LeaderElectionNode() :
  nodeDir(-1),
  nextNodeDir(-1),
  prevNodeDir(-1),
  particle(nullptr) {}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::activate() {
  nodeState = particle->state;
  if (nodeState == State::StretchExpansion) {
    if (predecessor == nullptr) {
      // This node is the head of a stretch
      // Process termination detection tokens
      if (hasNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Head has termination detection token...";
        std::shared_ptr<TerminationDetectionToken> token = peekNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
        qDebug() << "Peeked at the token...";
        if (token->counter != count) {
          // Different count -> send token back, no termination
          qDebug() << "Different count -> no termination";
          takeNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
          passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->traversed, 0, false));
        }
        else {
          // Same count, initiate lexicographic comparison if not started yet
          // At the end, send back if different string, send to next stretch if equal
          if (!lexCompInit) {
            // Initiate lexicographic comparison
            passNodeToken<LexCompInitToken>(nextNodeDir, std::make_shared<LexCompInitToken>(-1, count));
            lexCompInit = true;
            lexCompTryMerge = false;
          }
          qDebug() << "Same count; waiting for lexicographic comparison...";
        }
      }
      // Pass termination detection return tokens
      // If token intended for this stretch, process it
      if (hasNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir)) {
        std::shared_ptr<TerminationDetectionReturnToken> token = takeNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir);
        bool termination = token->termination;
        if (count != token->counter) {
          termination = false;
        }
        if (token->traversed + 1 >= token->ttl) {
          if (terminationDetectionInitiated) {
            qDebug() << "Received termination detection return token";
            if (termination) {
              if (count == 6) {
                // Single stretch with count 6 covering the outer border
                // Head becomes leader
                qDebug() << "Terminating...";
                particle->state = State::Leader;
                terminationDetectionInitiated = false;
                return;
              }
              else {
                // Multiple lexicographically equal stretches covering the border
                // Move to next state -> Trees to break symmetry
                qDebug() << "Trees to break symmetry";
                particle->state = State::Candidate;
                particle->tree = true;
                particle->headCount = count;
                terminationDetectionInitiated = false;
                return;
              }
            }
            else {
              qDebug() << "Not terminating...";
              terminationDetectionInitiated = false;
            }
          }
        }
        else {
          qDebug() << "Passing termination detection return token back";
          passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->ttl, token->traversed+1, termination));
        }
      }

      // Handle lexicographic comparison tokens
      // If ack token received -> start lexicographic comparison
      if (hasNodeToken<LexCompAckToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<LexCompAckToken>(nextNode()->prevNodeDir);
        if (lexCompInit && !lexicographicComparisonRight) {
          lexicographicComparisonRight = true;
          qDebug() << "Starting lexicographic comparison...";
        }
      }
      // If nack token received -> abort lexicographic comparison
      if (hasNodeToken<LexCompNackToken>(nextNode()->prevNodeDir)) {
        takeNodeToken<LexCompNackToken>(nextNode()->prevNodeDir);
        if (lexCompInit && !lexicographicComparisonRight) {
          lexCompInit = false;
        }
      }
      // If interrupt token received, cancel lexicographic comparison
      if (hasNodeToken<LexCompInterruptLeftToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Processing interrupt token from right...";
        takeNodeToken<LexCompInterruptLeftToken>(nextNode()->prevNodeDir);
        lexCompCleanUp();
        return;
      }
      if (hasNodeToken<LexCompInterruptRightToken>(prevNode()->nextNodeDir)) {
        qDebug() << "Processing interrupt token from left...";
        takeNodeToken<LexCompInterruptRightToken>(prevNode()->nextNodeDir);
        lexCompCleanUpForNbr();
        return;
      }
      // If init token received, send ack or nack token
      if (hasNodeToken<LexCompInitToken>(prevNode()->nextNodeDir)) {
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
      
      // =================================== Lexicographic comparison to clockwise adjacent stretch ============================================
      if (lexicographicComparisonRight) {
        // Request labels from the adjacent stretch
        if (!requestedNbrLabel) {
          passNodeToken<LexCompReqStretchLabelToken>(nextNodeDir, std::make_shared<LexCompReqStretchLabelToken>());
          requestedNbrLabel = true;
        }
        // Receive labels from the adjacent stretch after requesting them
        else if (!receivedNbrLabel) {
          if (hasNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir)) {
            std::shared_ptr<LexCompReturnStretchLabelToken> token = takeNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir);
            NbrLabel = token->value;
            receivedNbrLabel = true;
          }
          // Receive end of stretch tokens from adjacent stretch
          else if (hasNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir)) {
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
            passNodeToken<LexCompRetrieveNextLabelToken>(nextNodeDir, std::make_shared<LexCompRetrieveNextLabelToken>());
            requestedLabel = true;
          }
        }
        // Receive internal labels after requesting them
        else if (!receivedLabel) {
          if (hasNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir)) {
            std::shared_ptr<LexCompNextLabelToken> token = takeNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir);
            internalLabel = token->value;
            receivedLabel = true;
          }
          // Receive end of stretch tokens
          else if (hasNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir)) {
            takeNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir);
            internalLabel = 0;
            receivedLabel = true;
          }
        }
        // If both labels received, compare them
        if (receivedNbrLabel && receivedLabel) {
          qDebug() << "Comparing received labels: " + QString::number(internalLabel) + " & " + QString::number(NbrLabel);
          // If all labels thus far have been the same
          if (firstLargerLabel == 0) {
            // Then if the labels are different, remember which was larger
            if (internalLabel > NbrLabel) {
              qDebug() << "Set first larger label for self.";
              firstLargerLabel = 1;
            }
            else if (internalLabel < NbrLabel) {
              qDebug() << "Set first larger for neighbour.";
              firstLargerLabel = -1;
            }
          }
          if (firstLargerLabel != 0) {
            // Send back termination detection token -> no termination
            if (hasNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir)) {
              qDebug() << "Lexicographically inequal -> no termination";
              std::shared_ptr<TerminationDetectionToken> token = takeNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
              passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->traversed, 0, false));
            }
          }
          // If one of the labels is 0, then the stretch has exhausted all its labels
          // In this case, the other stretch is lexicographically larger
          if (internalLabel == 0 && NbrLabel != 0) {
            qDebug() << "Adjacent stretch is lexicographically larger";
            // Adjacent stretch is lexicographically larger -> no merge
            passNodeToken<LexCompInterruptRightToken>(nextNodeDir, std::make_shared<LexCompInterruptRightToken>());
            qDebug() << "Sent interrupt token";
            lexCompCleanUp();
            qDebug() << "Cleaned up";
          }
          else if (internalLabel != 0 && NbrLabel == 0) {
            // This stretch is lexicographically larger -> merge
            qDebug() << "This stretch is lexicographically larger";
            lexCompCleanUp();
            qDebug() << "Cleaned up";
            // Merge
            if (lexCompTryMerge) {
              qDebug() << "Attempting merge...";
              passNodeToken<LexCompAttemptMergeToken>(nextNodeDir, std::make_shared<LexCompAttemptMergeToken>(-1, count));
              mergePending = true;
              mergeAck = false;
              mergeDir = 1;
            }
          }
          // If both labels are 0, then both strings were of the same length
          else if (internalLabel == 0 && NbrLabel == 0) {
            // In this case, the string which had the first larger label is lexicographically larger
            qDebug() << "Stretches are of same length";
            if (firstLargerLabel == -1) {
              // Adjacent stretch is lexicographically larger -> no merge
              qDebug() << "But adjacent stretch is lexicographically larger";
              lexCompCleanUp();
              qDebug() << "Cleaned up";
            }
            else if (firstLargerLabel == 1) {
              // This stretch is lexicographically larger -> merge
              qDebug() << "But this stretch is lexicographically larger";
              lexCompCleanUp();
              qDebug() << "Cleaned up";
              // Merge
              if (lexCompTryMerge) {
                qDebug() << "Attempting merge...";
                passNodeToken<LexCompAttemptMergeToken>(nextNodeDir, std::make_shared<LexCompAttemptMergeToken>(-1, count));
                mergePending = true;
                mergeAck = false;
                mergeDir = 1;
              }
            }
            // If both strings were identical, then trigger termination detection
            else {
              qDebug() << "Stretches are lexicographically equal";
              lexCompCleanUp();
              qDebug() << "Cleaned up";
              /*
               * Termination detection
               * 
               * Termination detection happens in the counter-clockwise direction, opposite to the merges.
               * Whenever a stretch finds that it is lexicographically equal to its clockwise neighbour, 
               * termination detection is initiated. This is done by sending a TerminationDetectionToken
               * in the counter-clockwise direction. This token is given a TTL (Time To Live) value, 
               * indicating how many stretches it must traverse. This value is equal to 6/count.
               * Additionally, the token gets a 'traversed' value, which is incremented each time the token 
               * reaches the head node of a stretch. When traversed == ttl, the token will be sent back
               * after it has been processed.
               * 
               * Whenever a TerminationDetectionToken is received by any node, the node checks that
               * it doesn't also have a merge message. If it does, then the token will be sent back
               * with the 'termination' variable set to false.
               * 
               * When a head node receives the token, it compares its count to the 'counter' variable
               * stored in the token. If these are different, then the token will be sent back with 
               * 'termination' set to false.
               * If the counts are the same, however, it initiates lexicographic comparison to its
               * clockwise neighbour (if not already in progress). Only if they are lexicographically
               * equal, the token is sent to the next counter-clockwise adjacent stretch. Otherwise, 
               * The token is returned with the 'termination' variable set to false.
               * 
               * If ttl == traversed and the clockwise adjacent stretch is lexicographically equal,
               * then the token is returned with the 'termination' variable set to true.
               * 
               * When the initiator of termination detection receives a token back with 'termination'
               * set to true, then if count == 6, it is the only remaining stretch and the head will
               * become the leader. However, if count != 6, then the outer border is covered by 
               * multiple lexicographically equal stretches. In this case we move to the next state:
               * Trees to break symmetry.
               */

              if ((count == 1 || count == 2 || count == 3) && !terminationDetectionInitiated) {
                qDebug() << "Lexicographically equal -> starting termination detection...";
                passNodeToken<TerminationDetectionToken>(prevNodeDir, std::make_shared<TerminationDetectionToken>(-1, count, 6/count, 0));
                terminationDetectionInitiated = true;
              }
              else if (count == 6) {
                // Border covered by 1 stretch of count 6
                qDebug() << "Lexicographically equal with count 6 -> terminating...";
                particle->state = State::Leader;
                return;
              }

              // If termination detection token, send to next stretch
              // or if last stretch, return to initiator with termination set to true
              if (hasNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir)) {
                qDebug() << "Head has termination detection token AND lexicographically equal";
                std::shared_ptr<TerminationDetectionToken> token = takeNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
                if (token->traversed + 1 >= token->ttl) {
                  qDebug() << "Sending termination token back";
                  passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->traversed+1, 0, true));
                }
                else {
                  qDebug() << "Passing termination detection token to next head";
                  passNodeToken<TerminationDetectionToken>(prevNodeDir, std::make_shared<TerminationDetectionToken>(-1, token->counter, token->ttl, token->traversed+1));
                }
              }
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
            std::shared_ptr<LexCompNextLabelForNbrToken> token = takeNodeToken<LexCompNextLabelForNbrToken>(nextNode()->prevNodeDir);
            internalLabelForNbr = token->value;
            receivedLabelForNbr = true;
          }
          // Receive end of stretch tokens
          else if (hasNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir)) {
            takeNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir);
            internalLabelForNbr = 0;
            receivedLabelForNbr = true;
          }
        }
        // Respond to label request when ready
        if (receivedLabelRequestFromNbr && receivedLabelForNbr) {
          // If there was an internal label, send it
          qDebug() << "Sending label to neighbour: " + QString::number(internalLabelForNbr);
          if (internalLabelForNbr != 0) {
            passNodeToken<LexCompReturnStretchLabelToken>(prevNodeDir, std::make_shared<LexCompReturnStretchLabelToken>(-1, internalLabelForNbr));
            receivedLabelRequestFromNbr = false;
            requestedLabelForNbr = false;
            receivedLabelForNbr = false;
            internalLabelForNbr = 0;
          }
          // Otherwise signal end of stretch
          else {
            passNodeToken<LexCompEndOfNbrStretchToken>(prevNodeDir, std::make_shared<LexCompEndOfNbrStretchToken>());
            lexCompCleanUpForNbr();
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
          passNodeToken<LexCompInterruptLeftToken>(prevNodeDir, std::make_shared<LexCompInterruptLeftToken>());
          lexCompCleanUpForNbr();
        }
      }
      if (hasNodeToken<MergeRequestToken>(prevNode()->nextNodeDir)) {
        if (mergePending) {
          takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
        }
        else {
          takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
          passNodeToken<MergeAckToken>(prevNodeDir, std::make_shared<MergeAckToken>());
          mergePending = true;
          mergeAck = true;
          mergeDir = -1;
          // Interrupt lexicographic comparison if applicable
          if (lexCompInit) {
            passNodeToken<LexCompInterruptRightToken>(nextNodeDir, std::make_shared<LexCompInterruptRightToken>());
          }
          lexCompCleanUp();
        }
      }
      if (successor == nullptr) {
        // This node is also the tail of a stretch,
        // therefore it is a stretch of 1 node
        if (unaryLabel > 0 && !mergePending) {
          LeaderElectionNode* next = nextNode(true);
          if (unaryLabel > next->count && unaryLabel + next->count <= 6) {
            // Send a merge request
            passNodeToken<MergeRequestToken>(nextNodeDir, std::make_shared<MergeRequestToken>());
            mergePending = true;
            mergeAck = false;
            mergeDir = 1;
            // Interrupt lexicographic comparison if applicable
            if (lexicographicComparisonLeft) {
              passNodeToken<LexCompInterruptLeftToken>(prevNodeDir, std::make_shared<LexCompInterruptLeftToken>());
              lexCompCleanUpForNbr();
            }
          }
        }
        if (mergePending && mergeAck) {
          if (mergeDir == 1) {
            successor = nextNode(true);
            count += successor->count;
          }
          else {
            predecessor = prevNode(true);
          }
          mergePending = false;
          mergeAck = false;
        }
      }
      else {
        // Node is head but not tail of stretch
        if (hasNodeToken<MergeRequestToken>(prevNode()->nextNodeDir)) {
          if (!mergePending) {
            takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
            predecessor = prevNode(true);
            passNodeToken<MergeAckToken>(prevNodeDir, std::make_shared<MergeAckToken>());
            // Interrupt lexicographic comparison if applicable
            if (lexCompInit) {
              passNodeToken<LexCompInterruptRightToken>(nextNodeDir, std::make_shared<LexCompInterruptRightToken>());
            }
            lexCompCleanUp();
          }
          else {
            takeNodeToken<MergeRequestToken>(prevNode()->nextNodeDir);
            passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
          }
        }
        if (mergePending && mergeAck && mergeDir == -1) {
          predecessor = prevNode(true);
          mergePending = false;
          mergeAck = false;
        }
        else if (!mergePending) {
          if (!countSent && count > 0 && !lexCompInit) {
            passNodeToken<CountToken>(nextNodeDir, std::make_shared<CountToken>(-1, count));
            countSent = true;
          }
          else if (!lexCompInit) {
            if (hasNodeToken<CountReturnToken>(successor->prevNodeDir)) {
              std::shared_ptr<CountReturnToken> token = takeNodeToken<CountReturnToken>(successor->prevNodeDir);
              int value = token->value;
              countSent = false;
              if (count > 0 && count > value && count + value <= 6) {
                // Attempt to merge with the adjacent stretch
                passNodeToken<AttemptMergeToken>(nextNodeDir, std::make_shared<AttemptMergeToken>(-1, count));
                mergePending = true;
                mergeDir = 1;
                // Interrupt lexicographic comparison if applicable
                if (lexicographicComparisonLeft) {
                  // Lexicographic comparison was initiated by the counter-clockwise adjacent stretch
                  // Send interrupt token in this direction
                  passNodeToken<LexCompInterruptLeftToken>(prevNodeDir, std::make_shared<LexCompInterruptLeftToken>());
                  lexCompCleanUpForNbr();
                }
              }
              else if (count > 0 && count == value && count + value <= 6) {
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
                  lexCompTryMerge = true;
                }
              }
              else if ((count == 1 || count == 2 || count == 3 || count == 6) && count == value) {
                // Initialize lexicographic comparison
                // But do not merge, only start termination detection if strings are lexicographically equal
                passNodeToken<LexCompInitToken>(nextNodeDir, std::make_shared<LexCompInitToken>(-1, count));
                lexCompInit = true;
                lexCompTryMerge = false;
              }
            }
          }
        }
        else if (mergePending) {
          if (hasNodeToken<MergeNackToken>(successor->prevNodeDir)) {
            takeNodeToken<MergeNackToken>(successor->prevNodeDir);
            mergePending = false;
          }
          else if (hasNodeToken<MergeCountToken>(successor->prevNodeDir)) {
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
      // Pass termination detection tokens
      if (hasNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Tail node has termination detection token";
        std::shared_ptr<TerminationDetectionToken> token = takeNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
        bool hasMergeToken = false;
        if (hasNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<AttemptMergeToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeRequestToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeAckToken>(nextNode()->prevNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeCountToken>(nextNode()->prevNodeDir)) {
          hasMergeToken = true;
        }
        if (hasMergeToken) {
          if(token->traversed > 0) {
            passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->traversed+1, 0, false));
          }
        }
        else {
          qDebug() << "Passing termination detection token...";
          passNodeToken<TerminationDetectionToken>(prevNodeDir, std::make_shared<TerminationDetectionToken>(-1, token->counter, token->ttl, token->traversed));
        }
      }
      if (hasNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir)) {
        std::shared_ptr<TerminationDetectionReturnToken> token = takeNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir);
        passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->ttl, token->traversed, token->termination));
      }

      // Pass lexicographic comparison tokens
      if (hasNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir)) {
        // Receive cleanup tokens
        takeNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir);
        retrieved = false;
        // intercept label tokens and remove them
        while (hasNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir);
        }
        while (hasNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir);
        }
        while (hasNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir);
        }
        while (hasNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir);
        }
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
      if (hasNodeToken<LexCompInterruptLeftToken>(nextNode()->prevNodeDir)) {
        // Pass interrupt tokens in counter-clockwise direction
        takeNodeToken<LexCompInterruptLeftToken>(nextNode()->prevNodeDir);
        passNodeToken<LexCompInterruptLeftToken>(prevNodeDir, std::make_shared<LexCompInterruptLeftToken>());
      }
      if (hasNodeToken<LexCompInterruptRightToken>(predecessor->nextNodeDir)) {
        // Pass interrupt tokens in clockwise direction
        takeNodeToken<LexCompInterruptRightToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompInterruptRightToken>(nextNodeDir, std::make_shared<LexCompInterruptRightToken>());
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
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
        }
      }

      if (hasNodeToken<CountToken>(predecessor->nextNodeDir)) {
        std::shared_ptr<CountToken> token = takeNodeToken<CountToken>(predecessor->nextNodeDir);
        int value = token->value;
        count = value;
        LeaderElectionNode* headNbr = nextNode(true);
        passNodeToken<CountReturnToken>(prevNodeDir, std::make_shared<CountReturnToken>(-1, headNbr->count));
      }
      if (hasNodeToken<AttemptMergeToken>(predecessor->nextNodeDir)) {
        std::shared_ptr<AttemptMergeToken> token = takeNodeToken<AttemptMergeToken>(predecessor->nextNodeDir);
        int value = token->value;
        count = value;
        LeaderElectionNode* headNbr = nextNode(true);
        if (count > 0 && count > headNbr->count && count + headNbr->count <= 6) {
          passNodeToken<MergeRequestToken>(nextNodeDir, std::make_shared<MergeRequestToken>());
          mergePending = true;
          mergeDir = 1;
        }
        else {
          // Counts have changed since communication, abort merge.
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
        }
      }
      if (mergePending) {
        if (hasNodeToken<MergeAckToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<MergeAckToken>(nextNode()->prevNodeDir);
          successor = nextNode();
          mergePending = false;
          passNodeToken<MergeCountToken>(prevNodeDir, std::make_shared<MergeCountToken>(-1, successor->count));
        }
        else if (hasNodeToken<MergeNackToken>(nextNode()->prevNodeDir)) {
          takeNodeToken<MergeNackToken>(nextNode()->prevNodeDir);
          mergePending = false;
          passNodeToken<MergeNackToken>(prevNodeDir, std::make_shared<MergeNackToken>());
        }
      }
    }
    else if (predecessor != nullptr && successor != nullptr) {
      // Internal node
      // Pass termination detection tokens
      if (hasNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir)) {
        qDebug() << "Internal node has termination detection token";
        std::shared_ptr<TerminationDetectionToken> token = takeNodeToken<TerminationDetectionToken>(nextNode()->prevNodeDir);
        bool hasMergeToken = false;
        if (hasNodeToken<LexCompAttemptMergeToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<AttemptMergeToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeRequestToken>(predecessor->nextNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeAckToken>(successor->prevNodeDir)) {
          hasMergeToken = true;
        }
        if (hasNodeToken<MergeCountToken>(successor->prevNodeDir)) {
          hasMergeToken = true;
        }
        if (hasMergeToken) {
          if(token->traversed > 0) {
            passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->traversed+1, 0, false));
          }
        }
        else {
          qDebug() << "Passing termination detection token...";
          passNodeToken<TerminationDetectionToken>(prevNodeDir, std::make_shared<TerminationDetectionToken>(-1, token->counter, token->ttl, token->traversed));
        }
      }
      if (hasNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir)) {
        std::shared_ptr<TerminationDetectionReturnToken> token = takeNodeToken<TerminationDetectionReturnToken>(prevNode()->nextNodeDir);
        passNodeToken<TerminationDetectionReturnToken>(nextNodeDir, std::make_shared<TerminationDetectionReturnToken>(-1, token->counter, token->ttl, token->traversed, token->termination));
      }

      // Pass lexicographic comparison topkens
      if (hasNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir)) {
        // Pass cleanup tokens towards tail
        takeNodeToken<LexCompCleanUpToken>(predecessor->nextNodeDir);
        retrieved = false;
        passNodeToken<LexCompCleanUpToken>(nextNodeDir, std::make_shared<LexCompCleanUpToken>());
        // intercept label tokens and remove them
        while (hasNodeToken<LexCompReturnStretchLabelToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompReturnStretchLabelToken>(successor->prevNodeDir);
        }
        while (hasNodeToken<LexCompEndOfNbrStretchToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompEndOfNbrStretchToken>(successor->prevNodeDir);
        }
        while (hasNodeToken<LexCompNextLabelToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompNextLabelToken>(successor->prevNodeDir);
        }
        while (hasNodeToken<LexCompEndOfStretchToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompEndOfStretchToken>(successor->prevNodeDir);
        }
      }
      if (hasNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir)) {
        takeNodeToken<LexCompCleanUpForNbrToken>(predecessor->nextNodeDir);
        retrievedForNbr = false;
        passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
        // intercept label tokens for nbr and remove them
        while (hasNodeToken<LexCompNextLabelForNbrToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompNextLabelForNbrToken>(successor->prevNodeDir);
        }
        while (hasNodeToken<LexCompEndOfStretchForNbrToken>(successor->prevNodeDir)) {
          takeNodeToken<LexCompEndOfStretchForNbrToken>(successor->prevNodeDir);
        }
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
      if (hasNodeToken<LexCompInterruptLeftToken>(successor->prevNodeDir)) {
        // Pass interrupt tokens in counter-clockwise direction
        takeNodeToken<LexCompInterruptLeftToken>(successor->prevNodeDir);
        passNodeToken<LexCompInterruptLeftToken>(prevNodeDir, std::make_shared<LexCompInterruptLeftToken>());
      }
      if (hasNodeToken<LexCompInterruptRightToken>(predecessor->nextNodeDir)) {
        // Pass interrupt tokens in clockwise direction
        takeNodeToken<LexCompInterruptRightToken>(predecessor->nextNodeDir);
        passNodeToken<LexCompInterruptRightToken>(nextNodeDir, std::make_shared<LexCompInterruptRightToken>());
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

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::lexCompCleanUp() {
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
  while (hasNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompReturnStretchLabelToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompEndOfNbrStretchToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompNextLabelToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompEndOfStretchToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompRetrieveNextLabelToken>(prevNode()->nextNodeDir)) {
    takeNodeToken<LexCompRetrieveNextLabelToken>(prevNode()->nextNodeDir);
  }
  while (hasNodeToken<LexCompReqStretchLabelToken>(prevNode()->nextNodeDir)) {
    takeNodeToken<LexCompReqStretchLabelToken>(prevNode()->nextNodeDir);
  }
}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::lexCompCleanUpForNbr() {
  internalLabelForNbr = 0;
  receivedLabelRequestFromNbr = false;
  requestedLabelForNbr = false;
  receivedLabelForNbr = false;
  lexicographicComparisonLeft = false;
  retrievedForNbr = false;
  if (successor != nullptr) {
    passNodeToken<LexCompCleanUpForNbrToken>(nextNodeDir, std::make_shared<LexCompCleanUpForNbrToken>());
  }
  while (hasNodeToken<LexCompNextLabelForNbrToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompNextLabelForNbrToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir)) {
    takeNodeToken<LexCompEndOfStretchForNbrToken>(nextNode()->prevNodeDir);
  }
  while (hasNodeToken<LexCompRetrieveNextLabelForNbrToken>(prevNode()->nextNodeDir)) {
    takeNodeToken<LexCompRetrieveNextLabelForNbrToken>(prevNode()->nextNodeDir);
  }
}

void LeaderElectionStationaryDeterministicParticle::LeaderElectionNode::paintNode(
    const int color) {
  // paint a node
  if (nextNodeClone) {
    particle->borderHalfPointBetweenEdgeColorLabels.at(particle->localToGlobalDir(nodeDir)+6) = color;
  }
  else if (prevNodeClone) {
    particle->borderHalfPointBetweenEdgeColorLabels.at(particle->localToGlobalDir(nodeDir)) = color;
  }
  else {
    particle->borderPointBetweenEdgeColorLabels.at(particle->localToGlobalDir(nodeDir)) = color;
  }
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
peekNodeToken(int dir, bool checkClone) const {
  if (hasNodeToken<TokenType>(dir, false)) {
    auto prop = [dir,this](const std::shared_ptr<TokenType> token) {
      return token->origin == dir && token->destination == nodeDir;
    };
    return particle->peekAtToken<TokenType>(prop);
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
      return clone->peekNodeToken<TokenType>(cloneDir, false);
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
      return clone->peekNodeToken<TokenType>(cloneDir, false);
    }
    else {
      Q_ASSERT(false);
    }
  }
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

#include <string>
#include <fstream>
#include <sstream>
#include <QTextStream>

using namespace std;

LeaderElectionStationaryDeterministicSystem::LeaderElectionStationaryDeterministicSystem(int numParticles, QString fileName) {
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

      insert(new LeaderElectionStationaryDeterministicParticle(
      Node(x, y), -1, randDir(), *this,
      LeaderElectionStationaryDeterministicParticle::State::IdentificationLabeling));
    }

    file.close();

    outputPath = "../AmoebotSim/data/output/" + fileName.toStdString() + ".txt";

    out << "Particle system initialized from file." << endl;
    
    return;
  }

  randomPermutationScheduler = true;

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
    if (hp->state == LeaderElectionStationaryDeterministicParticle::State::Leader || hp->state == LeaderElectionStationaryDeterministicParticle::State::Finished) {
      if (outputPath != "") {
        ofstream file;
        file.open(outputPath);
        if (hp->state == LeaderElectionStationaryDeterministicParticle::State::Leader) {
          file << std::to_string(hp->head.x) << "," << std::to_string(hp->head.y);
        }
        else {
           file << "N/A";
        }

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