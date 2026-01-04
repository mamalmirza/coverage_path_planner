import numpy as np
from collections import deque
from algorithms import bfs_path

def stc_coverage(grid, robot_pos, state):
    """
    Spanning Tree Coverage (STC) algorithm.
    Wrapper that generates the full path once and then follows it.
    """
    # 1. Generate STC path if not already generated
    if not state.get("stc_generated"):
        full_path = generate_stc_path(grid, robot_pos)
        state["path"] = full_path
        state["stc_generated"] = True

    # 2. If path is empty (STC finished), switch to Cleanup Mode (Greedy/BFS)
    if not state["path"]:
        unvisited = np.argwhere(grid == 0)
        if len(unvisited) > 0:
            # Find nearest unvisited cell
            x, y = robot_pos
            distances = np.abs(unvisited - np.array([x, y])).sum(axis=1)
            target = tuple(unvisited[np.argmin(distances)])
            
            # Plan path to it
            cleanup_path = bfs_path(grid, (x, y), target)
            if len(cleanup_path) > 1:
                state["path"] = cleanup_path[1:]
    
    # 3. Execute path
    if state["path"]:
        return state["path"].pop(0)
        
    return robot_pos

def generate_stc_path(grid, start_pos):
    """
    Generates a coverage path using the STC algorithm.
    """
    rows, cols = grid.shape
    
    # 1. Define Mega-Cells (2x2 blocks)
    # Graph dimensions
    g_rows = rows // 2
    g_cols = cols // 2
    
    # helper to check if a mega cell is free (no obstacles)
    def is_free(gr, gc):
        # check the 2x2 block in the original grid
        r, c = gr * 2, gc * 2
        block = grid[r:r+2, c:c+2]
        return not np.any(block == 1) and not np.any(block == -1) # check for 1 or -1 (simulation uses different codes)

    # 2. Build Minimum Spanning Tree (MST)
    # We'll use a simple BFS/Prim's approach to build a tree on the graph of mega-cells
    start_gr, start_gc = start_pos[0] // 2, start_pos[1] // 2
    
    # Adjacency list for the tree: parent -> list of children
    # Actually, we just need edges or neighbors in the tree.
    tree_adj = {} # (r,c) -> list of neighbors (r,c) in the tree
    visited = set()
    queue = deque([(start_gr, start_gc)])
    visited.add((start_gr, start_gc))
    
    # Check if start is valid
    if not is_free(start_gr, start_gc):
        # Fallback if robot starts on obstacle/invalid (shouldn't happen in valid sim)
        return []

    while queue:
        curr = queue.popleft()
        cr, cc = curr
        
        # Try all 4 neighbors in random order for some variety, or fixed
        neighbors = [(cr-1, cc), (cr+1, cc), (cr, cc-1), (cr, cc+1)]
        # Filter valid and unvisited
        valid_neighbors = []
        for nr, nc in neighbors:
            if 0 <= nr < g_rows and 0 <= nc < g_cols and is_free(nr, nc):
                if (nr, nc) not in visited:
                    valid_neighbors.append((nr, nc))
        
        # For a spanning tree, we add ALL valid unvisited neighbors to the tree
        # (This is effectively building a BFS tree, which is a spanning tree)
        for nr, nc in valid_neighbors:
            visited.add((nr, nc))
            queue.append((nr, nc))
            
            # Add edge (undirected)
            tree_adj.setdefault(curr, []).append((nr, nc))
            tree_adj.setdefault((nr, nc), []).append(curr)

    # 3. Circumnavigate the Tree to generate path
    # We simulate walking around the "walls" of the tree.
    # To do this easily, we can treat each node as having 4 sub-cells.
    # We follow a specific order inside a mega-cell depending on entry/exit.
    
    # Easier implementation:
    # "Double" the resolution of the tree.
    # The edges in the tree block movement. Absence of edges allows movement?
    # Actually, STC path is often defined by dividing each node into 4 quadrants.
    # Let's use a "wall-following" around the tree edges.
    
    # We can create a high-res grid (the original grid) and place "walls" between
    # mega-cells that are NOT connected in the MST.
    # Then simply follow the wall on the right (or left).
    
    path = []
    
    # Current position in fine grid (relative to start)
    # We need a starting direction.
    curr_r, curr_c = start_pos
    
    # Initial direction: try to have wall on right.
    # For a counter-clockwise traversal, we keep wall on right.
    # Directions: 0=East, 1=South, 2=West, 3=North
    dr = [0, 1, 0, -1]
    dc = [1, 0, -1, 0]
    direction = 0 
    
    # We need to traverse the *entire* contour.
    # Detailed STC logic:
    # A Mega-cell (gr, gc) is divided into:
    # (2r, 2c)   (2r, 2c+1)
    # (2r+1, 2c) (2r+1, 2c+1)
    # The links in MST determine which internal boundaries are open.
    
    # Let's perform a recursive DFS walk on the tree, calculating coordinates.
    # Starting at the sub-cell of the root.
    
    # Find specific sub-cell within start mega-cell to start? 
    # Usually start at (0,0) of the mega-cell relative coords.
    
    final_path = []
    
    # Recursive function to traverse the tree nodes
    def traverse_node(node, incoming_dir):
        # node: (gr, gc)
        # incoming_dir: direction we entered from (0=from West, 1=from North, etc)
        # We need to map standard STC traversal.
        
        # Sub-cells in a 2x2 block (referenced by index 0,1,2,3)
        # 0:(0,0), 1:(0,1)
        # 3:(1,0), 2:(1,1)  <-- Counter-clockwise (CCW) order
        
        gr, gc = node
        top_left_r, top_left_c = gr*2, gc*2
        
        # Standard STC splits node into 4 sub-nodes.
        # But simpler: Just walk the sub-cells in CCW order.
        # If there is a neighbor in the tree in that direction, go there recursively,
        # then come back.
        
        # Sub-cell offsets for CCW traversal:
        # 1. Top-Left (0,0)
        # 2. Bot-Left (1,0)
        # 3. Bot-Right (1,1)
        # 4. Top-Right (0,1)
        
        # Wait, the order depends on where we entered.
        # Let's define the 4 sub-cells relative to (gr, gc):
        # TL=(0,0), BL=(1,0), BR=(1,1), TR=(0,1)
        
        # Neighbors of mega-cell:
        # West (-1, 0), South (1, 0), East (0, 1), North (-1, 0) ??
        # Let's stick to standard (dr, dc).
        
        # Neighbors of (gr, gc):
        # West:  (gr, gc-1)
        # South: (gr+1, gc)
        # East:  (gr, gc+1)
        # North: (gr-1, gc)
        
        # We want to visit sub-cells such that we are adjacent to the neighbor when we recurse.
        
        # Sequence of sub-cells for CCW Circumnavigation:
        # TL -> BL -> (South Neighbor) -> BL -> BR -> (East Neighbor) -> BR -> TR -> (North Neighbor) -> TR -> TL -> (West Neighbor)
        
        # Let's simplify.
        # We will generate a path of sub-cells.
        # We treat the node as having 4 internal points.
        
        # Coordinate definitions:
        # TL = (2*gr, 2*gc)
        # TR = (2*gr, 2*gc+1)
        # BL = (2*gr+1, 2*gc)
        # BR = (2*gr+1, 2*gc+1)
        
        # Depending on parent, we enter at a specific sub-cell.
        # Parent North -> Enter TL (from BL of parent?? No from BL of parent is impossible, parent is North)
        # Parent is North (gr-1, gc). We are South of Parent.
        # Parent enters us from its BL or BR?
        
        pass

    # Alternative Implementation:
    # Just treat the MST as a maze.
    # "Inflate" the obstacles: The walls between non-connected mega-cells are obstacles.
    # Use "Right Hand Rule" wall follower on the fine grid.
    
    # 1. Construct a virtual fine grid of walls.
    #    - Boundaries of grid are walls.
    #    - Boundaries of valid mega-cells that are NOT in MST edges are walls.
    #    - Internal boundaries of mega-cells are NEVER walls (we can always move inside a mega-cell).
    
    # 2. Run Wall Follower.
    
    # Function to check if a specific move in fine grid is allowed
    def can_move(curr_fine, next_fine):
        cr, cc = curr_fine
        nr, nc = next_fine
        
        # Boundary check
        if not (0 <= nr < rows and 0 <= nc < cols): return False
        if grid[nr, nc] == 1 or grid[nr, nc] == -1: return False
        
        # Determine which mega-cells these belong to
        curr_g = (cr // 2, cc // 2)
        next_g = (nr // 2, nc // 2)
        
        if curr_g == next_g:
            # Moving inside the same mega-cell -> Always allowed (since whole cell is free)
            return True
        else:
            # Moving between mega-cells -> Allowed ONLY if edge exists in MST
            # Check if edge exists
            neighbors = tree_adj.get(curr_g, [])
            return next_g in neighbors

    # Wall Follower Simulation
    # Start at robot_pos
    # Direction: Initially East (0, 1)
    
    path = []
    curr = tuple(start_pos)
    
    # Directions: Right, Down, Left, Up
    dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    curr_dir_idx = 0 # Facing Right
    
    # We want to follow the "right wall".
    # Logic:
    # 1. Try to turn RIGHT and move.
    # 2. If blocked, try to move STRAIGHT.
    # 3. If blocked, try to turn LEFT and move.
    # 4. If blocked, move BACK (turn 180).
    
    # But for full coverage, we typically want to hug the wall closely.
    # Standard Right-Hand Rule:
    # If right is open, turn right and move in.
    # Else if straight is open, move straight.
    # Else turn left.
    
    # To detect completion: Return to start?
    # STC path is a cycle. Using visited set is safer to stop infinite loops if something goes wrong.
    # However, since we want to cover everything, we stop when we return to start AND have covered expected area?
    # Or just run for a safely large number of steps or until we loop back to start with same direction.
    
    start_state = (curr, curr_dir_idx)
    MAX_STEPS = rows * cols * 4
    
    for _ in range(MAX_STEPS):
        # Right-hand rule logic
        
        # Relative directions
        right_dir_idx = (curr_dir_idx + 1) % 4
        left_dir_idx = (curr_dir_idx - 1) % 4
        back_dir_idx = (curr_dir_idx + 2) % 4
        
        # Check Right
        dr, dc = dirs[right_dir_idx]
        next_r, next_c = curr[0] + dr, curr[1] + dc
        if can_move(curr, (next_r, next_c)):
            curr = (next_r, next_c)
            curr_dir_idx = right_dir_idx
            path.append(curr)
        else:
            # Check Straight
            dr, dc = dirs[curr_dir_idx]
            next_r, next_c = curr[0] + dr, curr[1] + dc
            if can_move(curr, (next_r, next_c)):
                curr = (next_r, next_c)
                # dir unchanged
                path.append(curr)
            else:
                # Check Left (Turn Left)
                curr_dir_idx = left_dir_idx
                # Don't move yet, just turn. Next iteration will try "Right" (which is now our Straight)
                # Actually, standard algorithm usually moves immediately if possible.
                
                # Let's just create the path by pure simulation:
                # If can turn right, turn right and move
                # Else if can move straight, move straight
                # Else turn left (change dir, do not move)
                pass

        # Stop condition: Back at start?
        if len(path) > 1 and curr == tuple(start_pos) and curr_dir_idx == 0: # heuristic
             break
             
    # Clean up: Since the loop logic above was a bit mixed, let's rewrite the loop properly.
    path = []
    curr = tuple(start_pos)
    curr_dir_idx = 0 # Assumption: Init facing East
    
    # Force first move to be valid if possible
    # We need to find a wall to hug? STC creates walls.
    # Actually, if we are in a spanning tree, the "walls" are the boundaries of the tree.
    # We are inside the tree's "tube".
    
    visited_states = set()
    
    for _ in range(MAX_STEPS):
        state = (curr, curr_dir_idx)
        if state in visited_states and len(path) > 0:
            break
        visited_states.add(state)
        
        # Right hand rule
        # 1. Check Right
        right_idx = (curr_dir_idx + 1) % 4
        dr, dc = dirs[right_idx]
        next_r, next_c = curr[0] + dr, curr[1] + dc
        
        if can_move(curr, (next_r, next_c)):
            curr = (next_r, next_c)
            curr_dir_idx = right_idx
            path.append(curr)
            continue
            
        # 2. Check Straight
        dr, dc = dirs[curr_dir_idx]
        next_r, next_c = curr[0] + dr, curr[1] + dc
        
        if can_move(curr, (next_r, next_c)):
            curr = (next_r, next_c)
            path.append(curr)
            continue
            
        # 3. Turn Left
        curr_dir_idx = (curr_dir_idx - 1) % 4
        # Do not move this tick, just rotate
    
    return path
