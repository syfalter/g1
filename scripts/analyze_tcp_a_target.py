"""Read-only, target-scoped TCP analysis; exports new files, never uses ROS."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import uuid

import numpy as np

from tcp_a_core import export_session, load_session, now_iso, solve, write_new


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--target', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    original_hash = hashlib.sha256(args.input.read_bytes()).hexdigest()
    session = load_session(args.input)
    source_id = session['session_id']
    original_count = len(session['samples'])
    session['samples'] = [r for r in session['samples'] if r['target_id'] == args.target]
    if not session['samples']:
        parser.error('Target not found; no export created')
    provenance = dict(source_path=str(args.input.resolve()), source_sha256=original_hash,
                      source_session_id=source_id, source_sample_count=original_count,
                      target_id=args.target, selection='exact target_id; groups/exclusions unchanged',
                      method='All retained fit samples, unweighted SVD; validation never tunes TCP',
                      source_sample_ids=[r['id'] for r in session['samples']],
                      analysis_only=True, hardware_commands_sent=False)
    session['session_id'] = uuid.uuid4().hex
    session['derived_from'] = provenance
    session['audit'].append(dict(at=now_iso(), operation='derive_target_analysis', **provenance))
    result = solve(session)
    fits = [r for r in session['samples'] if not r['excluded'] and r['group'] == 'fit']
    candidates, failures = [], []
    for row in fits:
        temporary = copy.deepcopy(session)
        next(r for r in temporary['samples'] if r['id'] == row['id'])['excluded'] = True
        try:
            candidates.append(solve(temporary)['s_candidate_m'])
        except ValueError as error:
            failures.append(dict(sample_id=row['id'], reason=str(error)))
    sensitivity = dict(method='fit-only leave-one-out sensitivity, NOT used to select samples or candidate',
                       successful=len(candidates), failures=failures)
    if candidates:
        sensitivity.update(axis_range_mm=(np.ptp(candidates, axis=0)*1000).tolist(),
                           max_distance_from_full_fit_mm=float(np.max(np.linalg.norm(
                               np.array(candidates)-result['s_candidate_m'], axis=1))*1000))
    directory = export_session(session, args.output)
    write_new(directory/'provenance.json', json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    write_new(directory/'fit_sensitivity.json', json.dumps(sensitivity, ensure_ascii=False, indent=2)+'\n')
    snapshot = next(directory.glob('session_*.json'))
    recomputed = solve(load_session(snapshot))
    np.testing.assert_allclose(recomputed['s_candidate_m'], result['s_candidate_m'], rtol=0, atol=1e-12)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = [r for r in session['samples'] if not r['excluded']]
    before, after = [], []
    for row in rows:
        item = result['per_sample'][row['id']]
        before.append(np.linalg.norm(np.array(item['old_m'])-result['fit_old']['center_m'])*1000)
        after.append(np.linalg.norm(item['residual_m'])*1000)
    x = np.arange(len(rows))
    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    ax.bar(x-.18, before, .36, label='Original TCP', color='#3478b8')
    ax.bar(x+.18, after, .36, label='Candidate TCP', color='#3e9363')
    ax.set_xticks(x, [f"{r['group']}\n{r['id'][:6]}" for r in rows], fontsize=8)
    ax.set_ylabel('Distance to respective FIT center (mm)')
    ax.set_title(f'{args.target}: fit-only TCP estimate, unchanged held-out samples')
    ax.grid(axis='y', alpha=.2)
    ax.legend()
    fig.savefig(directory/'residual_comparison.png', dpi=160)
    plt.close(fig)
    assert hashlib.sha256(args.input.read_bytes()).hexdigest() == original_hash, 'Source changed during analysis'
    print(json.dumps(dict(export_directory=str(directory), source_unchanged=True,
                          fit_n=result['fit_new']['n'], validation=result['validation'],
                          s_model_mm=(np.array(result['s_model_m'])*1000).tolist(),
                          s_candidate_mm=(np.array(result['s_candidate_m'])*1000).tolist(),
                          delta_mm=(np.array(result['delta_tcp_m'])*1000).tolist(),
                          delta_norm_mm=float(np.linalg.norm(result['delta_tcp_m'])*1000),
                          fit_rms_before_mm=result['fit_old']['rms_m']*1000,
                          fit_rms_after_mm=result['fit_new']['rms_m']*1000,
                          fit_sensitivity=sensitivity), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
