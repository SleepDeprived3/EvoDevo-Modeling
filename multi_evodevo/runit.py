"""runit --- a module that integrates the other code into an
experimental run with data collection.

Author: Joshua Hawthorne-Madell
"""

import os
import multiprocessing as mp
from functools import partial
import sqlite3
import pickle
import csv
from operator import itemgetter
from numpy.random import RandomState
import cProfile

import part
import initiate
import develop
import blueprint
import export
import simulate
import selection


def do_cprofile(func):
    def profiled_func(*args, **kwargs):
        profile = cProfile.Profile()
        try:
            profile.enable()
            result = func(*args, **kwargs)
            profile.disable()
            return result
        finally:
            profile.print_stats()
    return profiled_func


def make_io_file(pop, repro_cond, build_cond, grav, io_dir='../io/'):
    mod_repro_cond = int(repro_cond * 10000)
    mod_build_cond = int(build_cond * 10000)
    cond_str = 'pop{}r{:04}b{:04}pop{}'.format(pop,
                                          mod_repro_cond,
                                          mod_build_cond,
                                          grav)
    io_file = ''.join((io_dir, cond_str))
    if not os.path.isdir(io_file):
        os.makedirs(io_file)
    return io_file


# Making a set of populations, and a function to grab them
def make_population_db(db='../data/population_genes.db', num=151):
    if not os.path.isdir(db[:8]):
        os.makedirs(db[:8])
    conn = sqlite3.connect(db)
    c = conn.cursor()
    for i in range(num):
        table = ''.join(('pop', str(i)))
        s = 'CREATE TABLE {} (id INT PRIMARY KEY, genome TEXT)'.format(table)
        c.execute(s)
    conn.commit()
    conn.close()


def fill_population_table(table, db='../data/population_genes.db'):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    pop = list()
    prng = RandomState(24)
    for i in range(60):
        genome = initiate.generate_genome(18000, prng)
        pop.append((i, genome))
    s = 'INSERT into pop{} VALUES (?, ?)'.format(table)
    c.executemany(s, pop)
    conn.commit()
    conn.close()


def make_filled_db(db='../data/population_genes.db', num=151):
    make_population_db(db, num)
    for i in range(num):
        fill_population_table(i, db)


def grab_pop_genome(table, agent, db='../data/population_genes.db'):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    t = (agent,)
    s = 'SELECT genome FROM pop{} WHERE id=?'.format(table)
    c.execute(s, t)
    genome = c.fetchone()
    conn.close()
    return genome[0]


# Create and manage simulation data
def make_sql_db(population, reproduction_cond,
                building_cond, generations, grav):
    mod_repro_cond = int(reproduction_cond * 10000)
    mod_build_cond = int(building_cond * 10000)
    the_file = '../data/pop{}r{:04}b{:04}g{}.db'.format(population,
                                                        mod_repro_cond,
                                                        mod_build_cond,
                                                        grav)
    if not os.path.isdir(the_file[:8]):
        os.makedirs(the_file[:8])
    while (os.path.isfile(the_file)):
        # print the_file
        index = the_file.find('.db')
        if the_file[index-1] == ')':
            if the_file[index-3:index-1] == '10':
                raise ValueError
            count = int(the_file[index-2]) + 1
            insert = ''.join((' (', str(count), ')'))
            the_file = ''.join((the_file[:index-4], insert, '.db'))
        else:
            the_file = ''.join((the_file[:index], ' (1).db'))
    # print the_file
    conn = sqlite3.connect(the_file)
    c = conn.cursor()
    for i in range(generations):
        s = '''CREATE TABLE gen{} (sim_num INT PRIMARY KEY,
        parent INT, fitness REAL, reproduction_error_rate REAL,
        buidling_error_rate REAL, germline_genes TEXT,
        somaline_genes TEXT)'''.format(i)
        c.execute(s)
    conn.commit()
    conn.close()
    return the_file


def write_to_sql_table(db, generation, data):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    s = '''INSERT INTO gen{} VALUES (?, ?, ?, ?, ?, ?, ?)'''.format(generation)
    c.execute(s, data)
    conn.commit()
    conn.close()


def grab_sim_selection_data(db, gen):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    s = 'SELECT sim_num, fitness, germline_genes FROM gen{}'.format(gen)
    c.execute(s)
    data = sorted(c.fetchall(),
                  key=itemgetter(1, 0))
    conn.close()
    fitness = [data[i][1] for i in range(len(data))]
    genomes = [(data[i][0], data[i][2]) for i in range(len(data))]
    while fitness[-1] == '-nan':
        print(fitness)
        fitness.pop()
        fitness.insert(0, 0)
        print(fitness)
        genomes.insert(0, genomes.pop())
    return [fitness, genomes]


# For examining various aspects of the data
def grab_a_genome(db, gen, agent, u=False):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    s = 'SELECT fitness, germline_genes, somaline_genes FROM gen{}'.format(gen)
    c.execute(s)
    data = sorted(c.fetchall(),
                  key=itemgetter(0))
    conn.close()
    if u:
        fitness = [data[i][0] for i in range(len(data))]
        germ_g = [data[i][1] for i in range(len(data))]
        soma_g = [data[i][2] for i in range(len(data))]
        while fitness[-1] == '-nan':
            fitness.pop()
            fitness.insert(0, 0)
            germ_g.insert(0, germ_g.pop())
            soma_g.insert(0, soma_g.pop())
        return [germ_g, soma_g]
    return data[agent][1:]


def grab_all_genomes(db):
    conn = sqlite3.connect(db)
    c = conn.cursor()
    sl = list()
    c.execute("SELECT name FROM sqlite_master WHERE type='table';")
    for i in range(len(c.fetchall())):
        sl.append('SELECT germline_genes, somaline_genes from gen{}'.format(i))
    genomes = list()
    for i in sl:
        c.execute(i)
        gen_genomes = c.fetchall()
        # gen_genomes = [gen_genomes[i][0] for i in len(gen_genomes)]
        genomes.append(gen_genomes)
    conn.close()
    return genomes


def compare_data(db1, db2, gen):
    conn = sqlite3.connect(db1)
    c = conn.cursor()
    s = 'SELECT sim_num, fitness, germline_genes, somaline_genes FROM gen{}'.format(gen)
    c.execute(s)
    data1 = sorted(c.fetchall(),
                   key=itemgetter(0))
    conn.close()
    conn = sqlite3.connect(db2)
    c = conn.cursor()
    s = 'SELECT sim_num, fitness, germline_genes, somaline_genes FROM gen{}'.format(gen)
    c.execute(s)
    data2 = sorted(c.fetchall(),
                   key=itemgetter(0))
    conn.close()
    # return [fitnesses, genomes]
    diff = list()
    for i in range(len(data1)):
        diff.append([data1[i][j] == data2[i][j] for j in range(1, 4)])
    for num, check in enumerate(diff):
        if all(check):
            print(data1[num][0], check)
        else:
            print(data1[num][0], check, data1[num][1], data2[num][1])


# Run and view one agent
def test_one_agent(pop, gen, build_er, repro_er, version, agent):
    db = '../data/pop{}r{:04}b{:04}.db'.format(pop,
                                               int(repro_er * 10000),
                                               int(build_er * 10000))
    if version != 0:
        index = db.find('.db')
        insert = ''.join((' (', str(version), ')'))
        db = ''.join((db[:index], insert, db[index:]))
    print(db)
    io_file = ''.join((make_io_file(pop, repro_er, build_er), '/'))
    genome, genome_to_build = grab_a_genome(db, gen, agent)
    filler_prng = RandomState(2)
    da_state = filler_prng.get_state()
    proto_parts, genome_to_build = initiate.setup_agent(genome_to_build,
                                                        0,
                                                        filler_prng)
    parts_developed = develop.update_cycles(proto_parts)
    frame_selection = develop.select_frame_parts(parts_developed)
    abort_data = (agent,
                  0,  # PARENT_HOLDER
                  0,  # fit = 0 when aborted
                  repro_er,
                  build_er,
                  genome,
                  genome_to_build)
    if any([i == [] for i in frame_selection]):
        return abort_data
    ann_selection = develop.select_ann_parts(parts_developed, frame_selection)
    if any([i == [] for i in ann_selection]):
        return abort_data
    blueprints = blueprint.all_parts_to_send(parts_developed, frame_selection,
                                             ann_selection)
    export.export_all(blueprints[0], blueprints[1], blueprints[2],
                      blueprints[3], blueprints[4], blueprints[5],
                      blueprints[6], agent, io_file)
    fitness = simulate.run_simulation(io_file, agent, True)
    data = (agent,
            0,  # PARENT_HOLDER
            fitness,
            repro_er,
            build_er,
            genome,
            genome_to_build)
    return da_state


def grab_data(data_file):
    with open(data_file, 'r') as fr:
        lines = [line.strip('\n') for line in fr]
    return lines


def write_data(data_file, data, part=False):
    with open(data_file, 'w+') as fw:
        for line in data:
            if not part:
                fw.write(str(line))
                fw.write('\n')
            else:
                fw.write(str(line.characteristics))
                fw.write('\n')


def check_data(og, cp):
    dbs_with_errors = list()
    for dirname, dirs, files in os.walk(og):
        for filename in files:
            og_file = os.path.join(dirname, filename)
            og_data = grab_data(og_file)
            found = False
            for dirname2, dirs2, files2 in os.walk(cp):
                for filename2 in files2:
                    if filename == filename2:
                        found = True
                        cp_file = os.path.join(dirname2, filename2)
                        cp_data = grab_data(cp_file)
                        if cp_data != og_data:
                            print("ERROR with ", og_file)
                            dbs_with_errors.append(og_file[-14:-4])
            if not found:
                print("Could not find other ", og_file)
    dbs_with_errors = sorted(set(dbs_with_errors))
    print(len(dbs_with_errors), dbs_with_errors)


# Single simulation. Will be multiprocessed
def run_one(sim_num, seeds, all_genomes, gen, build_er, repro_er, io_file, db, grav):
    agent_prng = RandomState(seeds[sim_num])
    genome_info = all_genomes[sim_num]
    parent = genome_info[0]
    genome = genome_info[1]
    proto_parts, genome_to_build = initiate.setup_agent(genome,
                                                        build_er,
                                                        agent_prng)
    parts_developed = develop.update_cycles(proto_parts)
    frame_selection = develop.select_frame_parts(parts_developed)
    abort_data = (sim_num,
                  parent,
                  0,  # fit = 0 when aborted
                  repro_er,
                  build_er,
                  genome,
                  genome_to_build)
    if any([i == [] for i in frame_selection]):
        return write_to_sql_table(db, gen, abort_data)
    ann_selection = develop.select_ann_parts(parts_developed,
                                             frame_selection)
    if any([i == [] for i in ann_selection]):
        return write_to_sql_table(db, gen, abort_data)
    blueprints = blueprint.all_parts_to_send(parts_developed,
                                             frame_selection,
                                             ann_selection)
    export.export_all(blueprints[0], blueprints[1], blueprints[2],
                      blueprints[3], blueprints[4], blueprints[5],
                      blueprints[6], sim_num, io_file)
    fitness = simulate.run_simulation(io_file, sim_num, grav=grav)
    data = (sim_num,
            parent,
            fitness,
            repro_er,
            build_er,
            genome,
            genome_to_build)
    return write_to_sql_table(db, gen, data)


def run_generations(reproduction_error_rate, build_error_rate,
                    pop_num, grav=-9.81, generations=100, agents=60):
    main_prng = RandomState(42)

    db_name = make_sql_db(pop_num,
                          reproduction_error_rate,
                          build_error_rate,
                          generations, 
                          grav)
    io_file = ''.join((make_io_file(pop_num,
                                    reproduction_error_rate,
                                    build_error_rate,
                                    grav),
                       '/'))
    initial_genomes = list()
    for i in range(agents):
        initial_genomes.append((i,
                                grab_pop_genome(pop_num,
                                                i)))
    for generation in range(generations):
        # print 'Gen:', generation, 'Genomes:', len(initial_genomes)
        # print 'Length of genomes:'
        # for i in initial_genomes:
        #     if len(i[1]) < 10:
        #         # pdb.set_trace()
        #         print
        #         print i[1]
        #     else:
        #         print len(i[1]),
        # print; print
        agent_prng_seeds = main_prng.randint(0, 1000000, agents+1)
        setup_data = [generation, reproduction_error_rate, build_error_rate, grav]
        wrap_run_one = partial(run_one,
                               seeds=agent_prng_seeds,
                               all_genomes=initial_genomes,
                               gen=generation,
                               build_er=build_error_rate,
                               repro_er=reproduction_error_rate,
                               db=db_name,
                               io_file=io_file,
                               grav=grav)
        sim_pool = mp.Pool(mp.cpu_count())
        sim_pool.map(wrap_run_one, range(agents))
        sim_pool.close()
        sim_pool.join()
        fit_data, selection_genomes = grab_sim_selection_data(db_name,
                                                              generation)
        # print fit_data
        # print ('Fits:', len(fit_data), 'Selection Genomes:',
        #        len(selection_genomes))
        # print 'Length of selection genomes:'
        # for i in selection_genomes:
        #     if len(i[1]) < 10:
        #         # pdb.set_trace()
        #         print
        #         print len(i[1]), i[1]
        #     else:
        #         print len(i[1]),
        # print; print
        initial_genomes = selection.next_generation(selection_genomes,
                                                    fit_data,
                                                    reproduction_error_rate,
                                                    agent_prng_seeds[-1])
        # print 'Length of Selected Genomes:'
        # for i in initial_genomes:
        #     if len(i[1]) < 10:
        #         # pdb.set_trace()
        #         print
        #         print len(i[1]), i[1]
        #     else:
        #         print len(i[1]),
        # print; print
        del(selection_genomes)
        del(fit_data)
        # print "Done with generation", generation
    cond_str = 'prng_pop{}r{:04}b{:04}.p'.format(pop_num,
                                                 int(reproduction_error_rate *
                                                     10000),
                                                 int(build_error_rate * 10000))
    pickle_dir = '../data/prng_states/'
    if not os.path.isdir(pickle_dir):
        os.makedirs(pickle_dir)
    pickle_file = ''.join((pickle_dir, cond_str))
    with open(pickle_file, 'wb+') as pfw:
        pickle.dump(main_prng, pfw)
    return db_name


def export_db_to_csv(db_file):
    """Export all tables from SQLite database to CSV files."""
    con = sqlite3.connect(db_file)
    con.row_factory = sqlite3.Row
    cursor = con.cursor()
    
    # Get all table names
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
    tables = cursor.fetchall()
    
    if not tables:
        print(f"No tables found in {db_file}")
        con.close()
        return
    
    db_dir = os.path.dirname(db_file)
    db_name = os.path.basename(db_file).replace('.db', '')
    
    for table in tables:
        table_name = table[0]
        csv_file = os.path.join(db_dir, f"{db_name}_{table_name}.csv")
        
        # Get all data from table
        cursor.execute(f"SELECT * FROM {table_name}")
        rows = cursor.fetchall()
        
        if rows:
            # Get column names
            col_names = [description[0] for description in cursor.description]
            
            # Write to CSV
            with open(csv_file, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=col_names)
                writer.writeheader()
                for row in rows:
                    writer.writerow(dict(row))
            print(f"  ✓ Exported {table_name} -> {csv_file}")
        else:
            print(f"  ⚠ {table_name} is empty, skipping CSV export")
    
    con.close()


@do_cprofile
def main():
    # Ensure required directories exist
    data_dir = '../data/'
    io_dir = '../io/'
    if not os.path.isdir(data_dir):
        os.makedirs(data_dir)
        print(f"✓ Created directory: {data_dir}")
    if not os.path.isdir(io_dir):
        os.makedirs(io_dir)
        print(f"✓ Created directory: {io_dir}")
    
    if not os.path.isfile('../data/population_genes.db'):
        make_filled_db()
    pop_num = int(input("What population should be run? "))
    gen_num = int(input("How many generations should be run? "))
    #num_trials = int(input("How many independent trials (reps) per condition? "))
    thing_to_test = str(input("What type of experiment do you want to run? (Select one of the following)\n1. Gravity\n2. Error Rate\n")).lower()
    databases_to_export = []  # Track databases for CSV export
    
    if thing_to_test == "1":
        rep_er = float(input(
            "What reproduction error condition? "))
        build_er = float(input(
            "What build error condition? "))
        # Run 5 generations with incrementally increasing gravity
        gravity_values = [-4, -7, -10, -13, -16]
        for grav_val in gravity_values:
            print("Gravity: ", grav_val)
            db_file = run_generations(rep_er, build_er, pop_num, grav=grav_val, generations=gen_num)
            databases_to_export.append(db_file)
        
        # Export all databases to CSV files
        print("\n" + "="*70)
        print("EXPORTING DATABASES TO CSV FILES")
        print("="*70)
        for db_file in databases_to_export:
            print(f"\nExporting {os.path.basename(db_file)}...")
            export_db_to_csv(db_file)
        print("\n" + "="*70)
        print("CSV EXPORT COMPLETE")
        print("="*70)
        return 0
        
        
    elif thing_to_test == "2":
        rep_cond_start = int(input(
            "Start with which reproduction error condition? "))
        build_cond_start = int(input(
            "Start with which build error condition? "))
        rep_cond_end = int(input(
            "End with which reproduction error condition? "))
        build_cond_end = int(input(
            "End with which build error condition? "))
        for rep_er_cond in range(rep_cond_start, rep_cond_end+1, 5):
            rep_er = rep_er_cond / 10000.
            for build_er_cond in range(build_cond_start, build_cond_end+1, 5):
                build_er = build_er_cond / 10000.
                print("Start condition ", [rep_er, build_er])
                db_file = run_generations(rep_er, build_er, pop_num, generations=gen_num)
                databases_to_export.append(db_file)
        
        # Export all databases to CSV files
        print("\n" + "="*70)
        print("EXPORTING DATABASES TO CSV FILES")
        print("="*70)
        for db_file in databases_to_export:
            print(f"\nExporting {os.path.basename(db_file)}...")
            export_db_to_csv(db_file)
        print("\n" + "="*70)
        print("CSV EXPORT COMPLETE")
        print("="*70)
        return 0
    else:
        return "Error. Stopping experiment"
    


# Script to run 1 population through all conditions, serially
if __name__ == '__main__':
    main()
