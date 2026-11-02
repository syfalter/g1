"""Versioned business recipes referencing immutable actions, not new curves."""
import argparse
from itertools import product
from pathlib import Path

from bundle import digest, write


def gear_recipe(parameters):
    expected = {'startPosition', 'roundTrips', 'finishPosition', 'exitToAbove'}
    if set(parameters) != expected:
        raise ValueError('Gear sequence requires exactly: ' + ', '.join(sorted(expected)))
    start, finish = parameters['startPosition'], parameters['finishPosition']
    rounds, leave = parameters['roundTrips'], parameters['exitToAbove']
    if start not in ('LOWER', 'UPPER') or finish not in ('LOWER', 'UPPER'):
        raise ValueError('Gear start/finish positions must be LOWER or UPPER')
    if type(rounds) is not int or not 0 <= rounds <= 1000:
        raise ValueError('roundTrips must be an integer from 0 to 1000')
    if type(leave) is not bool:
        raise ValueError('exitToAbove must be a boolean')
    lower = 'gear_lower_to_upper'
    upper = 'gear_upper_to_lower'
    tail = [] if start == finish else [lower if start == 'LOWER' else upper]
    if leave:
        tail.append('gear_' + finish.lower() + '_to_above')
    return dict(schema=1, kind='GEAR_SEQUENCE',
                entry=['gear_above_to_' + start.lower()],
                repeat=[lower, upper] if start == 'LOWER' else [upper, lower],
                repetitions=rounds, tail=tail,
                finalPosition='ABOVE' if leave else finish,
                counting='complete-round-trips-then-final-transfer',
                requiresStartAt='ABOVE', trajectoryPolicy='existing-recordings-and-local-start-alignment')


def actions(recipe):
    yield from recipe['entry']
    for _ in range(recipe['repetitions']):
        yield from recipe['repeat']
    yield from recipe['tail']


def required(recipe):
    return set(recipe['entry'] + recipe['tail'] + (recipe['repeat'] if recipe['repetitions'] else []))


def describe(recipe, release):
    names = required(recipe)
    selected = release['payload']['actions']
    missing = sorted(names - selected.keys())
    count = len(recipe['entry']) + len(recipe['repeat']) * recipe['repetitions'] + len(recipe['tail'])
    references = {name: selected[name]['source_plan_sha256'] for name in sorted(names & selected.keys())}
    return dict(recipe=recipe, recipeSha256=digest(recipe), releaseSha256=release['sha256'],
                releaseVersion=release['payload']['version'], selectedPlanHashes=references,
                missingActions=missing, configured=not missing, actionCount=count,
                hardwareCommandsSent=0, physicalSequenceVerified=False)


def alternatives(release):
    result = {}
    for start, finish, leave, rounds in product(('LOWER', 'UPPER'), ('LOWER', 'UPPER'), (False, True), (0, 1, 3)):
        parameters = dict(startPosition=start, roundTrips=rounds, finishPosition=finish, exitToAbove=leave)
        name = '%s-%s-%s-%s' % (start.lower(), rounds, finish.lower(), 'exit' if leave else 'hold')
        result[name] = dict(parameters=parameters, **describe(gear_recipe(parameters), release))
    for count in (1, 3):
        recipe = dict(schema=1, kind='NWS_CYCLE', entry=[], repeat=['dring_left', 'dring_right'],
                      repetitions=count, tail=[], finalPosition='ABOVE',
                      counting='one-left-plus-one-right', trajectoryPolicy='unchanged-full-actions')
        result['dring-pairs-' + str(count)] = dict(parameters={'count':count}, **describe(recipe, release))
    return result


if __name__ == '__main__':
    from releases import load_release
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    write(args.output, alternatives(load_release(args.release)))
